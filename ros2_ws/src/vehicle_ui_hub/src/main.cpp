#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QScroller>
#include <QShortcut>
#include <QStatusBar>
#include <QStringList>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QVector3D>
#include <QWheelEvent>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kHubPrefix = "/lidar_perception/hub";

struct Camera
{
  const char * key;
  const char * title;
  const char * color;
  const char * display;
};

const std::array<Camera, 4> kCameras{{
  {"front", "FRONT", "#4fc1e6", "Front ZED"},
  {"left", "LEFT", "#78d3a7", "Left ZED"},
  {"rear", "REAR", "#f0ad69", "Rear ZED"},
  {"right", "RIGHT", "#b49ae8", "Right ZED"},
}};

QLabel * label(const QString & text, const char * name = nullptr)
{
  auto * result = new QLabel(text);
  if (name) {
    result->setObjectName(name);
  }
  return result;
}

QIcon navigationIcon(bool calibration)
{
  QPixmap icon(24, 24);
  icon.fill(Qt::transparent);
  QPainter painter(&icon);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(QColor("#eeeeee"), 1.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  if (calibration) {
    painter.drawLine(3, 6, 21, 6);
    painter.drawLine(3, 12, 21, 12);
    painter.drawLine(3, 18, 21, 18);
    painter.setBrush(QColor("#eeeeee"));
    painter.drawEllipse(QPointF(8, 6), 2, 2);
    painter.drawEllipse(QPointF(16, 12), 2, 2);
    painter.drawEllipse(QPointF(10, 18), 2, 2);
  } else {
    painter.drawEllipse(QPointF(12, 12), 8.5, 8.5);
    painter.drawEllipse(QPointF(12, 12), 3.5, 3.5);
    painter.drawLine(12, 1, 12, 4);
    painter.drawLine(12, 20, 12, 23);
  }
  return QIcon(icon);
}

// A clearly marked illustration for checking the layout without ROS or OpenGL.
class PreviewScene : public QWidget
{
public:
  explicit PreviewScene(QWidget * parent = nullptr) : QWidget(parent)
  {
    setCursor(Qt::OpenHandCursor);
    const std::array<QColor, 5> colors{{QColor("#d2e0f0"), QColor("#4fc1e6"),
      QColor("#78d3a7"), QColor("#f0ad69"), QColor("#b49ae8")}};
    for (int i = 0; i < 4200; ++i) {
      const float a = static_cast<float>((i * 137) % 997) / 997.0F;
      const float b = static_cast<float>((i * 311) % 991) / 991.0F;
      QVector3D point;
      switch (i % 4) {
        case 0: point = QVector3D(-5.0F + 10.0F * a, -5.0F + 10.0F * b, 0.0F); break;
        case 1: point = QVector3D(-5.0F + 10.0F * a, 4.5F, 3.2F * b); break;
        case 2: point = QVector3D(-4.5F, -5.0F + 10.0F * a, 3.2F * b); break;
        default: point = QVector3D(3.5F + 0.8F * a, -3.5F + 1.0F * b,
            1.8F * static_cast<float>((i * 97) % 983) / 983.0F); break;
      }
      const std::size_t layer = static_cast<std::size_t>(i) % colors.size();
      QColor color = colors[layer];
      color.setAlpha(120 + (i % 120));
      points_.push_back({point, color, layer});
    }
  }

  void setLayerVisible(const QString & display, bool visible)
  {
    if (display == "LiDAR") {
      visible_layers_[0] = visible;
    } else {
      for (std::size_t index = 0; index < kCameras.size(); ++index) {
        if (display == kCameras[index].display) {
          visible_layers_[index + 1] = visible;
          break;
        }
      }
    }
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor("#111111"));
    const QVector3D target(0, 0, 0.5F);
    const QVector3D eye = target + QVector3D(
      distance_ * std::cos(pitch_) * std::cos(yaw_),
      distance_ * std::cos(pitch_) * std::sin(yaw_), distance_ * std::sin(pitch_));
    const QVector3D forward = (target - eye).normalized();
    const QVector3D right = QVector3D::crossProduct(forward, QVector3D(0, 0, 1)).normalized();
    const QVector3D up = QVector3D::crossProduct(right, forward).normalized();
    const float focal = static_cast<float>(std::min(width(), height())) * 0.88F;
    const QPointF center(width() * 0.50, height() * 0.53);
    auto project = [&](const QVector3D & point) {
      const QVector3D relative = point - eye;
      const float depth = std::max(0.1F, QVector3D::dotProduct(relative, forward));
      return center + QPointF(focal * QVector3D::dotProduct(relative, right) / depth,
        -focal * QVector3D::dotProduct(relative, up) / depth);
    };
    p.setPen(QPen(QColor(100, 100, 100, 85), 1));
    for (int index = -6; index <= 6; ++index) {
      p.drawLine(project(QVector3D(index, -6, 0)), project(QVector3D(index, 6, 0)));
      p.drawLine(project(QVector3D(-6, index, 0)), project(QVector3D(6, index, 0)));
    }
    for (const auto & point : points_) {
      if (visible_layers_[point.layer]) {
        p.setPen(QPen(point.color, 1.5));
        p.drawPoint(project(point.position));
      }
    }
    const std::array<QVector3D, 8> car{{
      {-0.7F, -0.45F, 0.10F}, {1.2F, -0.45F, 0.10F}, {1.2F, 0.45F, 0.10F}, {-0.7F, 0.45F, 0.10F},
      {-0.7F, -0.45F, 0.70F}, {1.2F, -0.45F, 0.70F}, {1.2F, 0.45F, 0.70F}, {-0.7F, 0.45F, 0.70F}}};
    p.setPen(QPen(QColor("#b6d5df"), 2));
    for (int index = 0; index < 4; ++index) {
      p.drawLine(project(car[index]), project(car[(index + 1) % 4]));
      p.drawLine(project(car[index + 4]), project(car[((index + 1) % 4) + 4]));
      p.drawLine(project(car[index]), project(car[index + 4]));
    }
    p.setPen(QColor("#8c8c8c"));
    p.drawText(QRect(24, height() - 40, width() - 48, 20),
      Qt::AlignLeft | Qt::AlignVCenter, "SIMULATED POINTS");
  }

  void mousePressEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::LeftButton) {
      last_mouse_ = event->pos();
      setCursor(Qt::ClosedHandCursor);
    }
  }

  void mouseMoveEvent(QMouseEvent * event) override
  {
    if (event->buttons() & Qt::LeftButton) {
      const QPoint delta = event->pos() - last_mouse_;
      yaw_ += static_cast<float>(delta.x()) * 0.008F;
      pitch_ = std::clamp(pitch_ + static_cast<float>(delta.y()) * 0.006F, 0.12F, 1.45F);
      last_mouse_ = event->pos();
      update();
    }
  }

  void mouseReleaseEvent(QMouseEvent *) override { setCursor(Qt::OpenHandCursor); }

  void wheelEvent(QWheelEvent * event) override
  {
    distance_ = std::clamp(distance_ * std::pow(0.85F, event->angleDelta().y() / 120.0F), 5.0F, 30.0F);
    update();
  }

private:
  struct Point
  {
    QVector3D position;
    QColor color;
    std::size_t layer;
  };
  std::vector<Point> points_;
  std::array<bool, 5> visible_layers_{{true, true, true, true, true}};
  QPoint last_mouse_;
  float yaw_ = 0.9F;
  float pitch_ = 0.60F;
  float distance_ = 12.0F;
};

class CameraButton : public QPushButton
{
public:
  CameraButton(const QString & title, const QColor & color)
  : title_(title), color_(color)
  {
    setMinimumHeight(92);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setCursor(Qt::PointingHandCursor);
    setSummary("Waiting to start");
  }

  void setSummary(const QString & summary)
  {
    summary_ = summary;
    setAccessibleName(title_ + " camera: " + summary);
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const QColor fill = isDown() ? QColor("#343434") :
      (isEnabled() && underMouse() ? QColor("#303030") : QColor("#242424"));
    painter.setBrush(fill);
    painter.setPen(QPen(hasFocus() ? QColor("#b0b0b0") : QColor("#424242"), 1));
    painter.drawRoundedRect(rect().adjusted(1, 1, -1, -1), 6, 6);
    painter.setPen(Qt::NoPen);
    painter.setBrush(color_);
    painter.drawEllipse(QPointF(19, 27), 4, 4);
    QFont heading = font();
    heading.setPixelSize(15);
    heading.setWeight(QFont::DemiBold);
    painter.setFont(heading);
    painter.setPen(isEnabled() ? QColor("#f2f2f2") : QColor("#b1b1b1"));
    painter.drawText(QRect(31, 12, width() - 43, 30), Qt::AlignVCenter, title_);
    QFont detail = font();
    detail.setPixelSize(12);
    painter.setFont(detail);
    painter.setPen(QColor("#aaaaaa"));
    const QRect detail_rect(15, 52, width() - 30, 22);
    painter.drawText(detail_rect, Qt::AlignVCenter,
      painter.fontMetrics().elidedText(summary_, Qt::ElideRight, detail_rect.width()));
  }

private:
  QString title_;
  QColor color_;
  QString summary_;
};

class HubWindow : public QMainWindow
{
public:
  HubWindow(QWidget * viewport, rviz_common::VisualizationFrame * rviz,
    rclcpp::Node::SharedPtr node, bool preview)
  : rviz_(rviz), node_(std::move(node)), preview_(preview)
  {
    preview_scene_ = preview ? static_cast<PreviewScene *>(viewport) : nullptr;
    setWindowTitle("1811 Vehicle Hub");
    resize(1440, 900);
    setMinimumSize(800, 560);
    setStyleSheet(styles());
    auto * root = new QWidget;
    root->setObjectName("root");
    auto * outer = new QVBoxLayout(root);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);
    outer->addWidget(makeHeader());

    auto * body = new QWidget;
    auto * body_layout = new QHBoxLayout(body);
    body_layout->setContentsMargins(0, 0, 0, 0);
    body_layout->setSpacing(0);

    auto * main = new QWidget;
    auto * main_layout = new QVBoxLayout(main);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);
    auto * scene_bar = new QFrame;
    scene_bar->setObjectName("sceneBar");
    scene_bar->setFixedHeight(54);
    auto * scene_layout = new QHBoxLayout(scene_bar);
    scene_layout->setContentsMargins(26, 0, 26, 0);
    scene_layout->addWidget(label("Surround view", "sceneTitle"));
    scene_layout->addStretch();
    scene_status_ = label(preview_ ? "Simulation" : "LiDAR status unavailable", "sceneStatus");
    scene_layout->addWidget(scene_status_);
    main_layout->addWidget(scene_bar);

    auto * view_frame = new QFrame;
    view_frame->setObjectName("viewFrame");
    auto * view_layout = new QVBoxLayout(view_frame);
    view_layout->setContentsMargins(0, 0, 0, 0);
    view_layout->addWidget(viewport);
    main_layout->addWidget(view_frame, 1);
    main_layout->addWidget(makeLegend());
    body_layout->addWidget(main, 1);
    drawer_ = makeCalibrationDrawer();
    drawer_area_ = new QScrollArea;
    drawer_area_->setObjectName("drawerArea");
    drawer_area_->setFixedWidth(480);
    drawer_area_->setWidgetResizable(true);
    drawer_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    drawer_area_->setWidget(drawer_);
    QScroller::grabGesture(drawer_area_->viewport(), QScroller::TouchGesture);
    drawer_area_->hide();
    body_layout->addWidget(drawer_area_);
    outer->addWidget(body, 1);
    outer->addWidget(makeNavigation());
    setCentralWidget(root);

    auto * fullscreen_shortcut = new QShortcut(QKeySequence(Qt::Key_F11), this);
    connect(fullscreen_shortcut, &QShortcut::activated, this, [this]() { toggleFullscreen(); });
    auto * escape_shortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(escape_shortcut, &QShortcut::activated, this, [this]() {
      if (isFullScreen()) { toggleFullscreen(); }
    });

    if (node_) {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        std::string(kHubPrefix) + "/status", qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          const QString data = QString::fromStdString(msg->data);
          QMetaObject::invokeMethod(this, [this, data]() { receiveStatus(data); }, Qt::QueuedConnection);
        });
      auto * watchdog = new QTimer(this);
      connect(watchdog, &QTimer::timeout, this, [this]() {
        if (!last_status_.isValid() || last_status_.elapsed() > 3000) {
          connection_->setText("Registration offline");
          scene_status_->setText("LiDAR status unavailable");
          begin_->setEnabled(false);
          for (auto & entry : camera_buttons_) {
            entry.second->setEnabled(false);
          }
          accept_->setEnabled(false);
        }
      });
      watchdog->start(1000);
    } else {
      populatePreview();
    }
  }

  void showCalibration()
  {
    calibration_nav_->click();
  }

protected:
  void resizeEvent(QResizeEvent * event) override
  {
    QMainWindow::resizeEvent(event);
    if (drawer_area_) {
      drawer_area_->setFixedWidth(std::clamp(width() * 38 / 100, 360, 560));
    }
  }

  void changeEvent(QEvent * event) override
  {
    QMainWindow::changeEvent(event);
    if (fullscreen_button_ && event->type() == QEvent::WindowStateChange) {
      fullscreen_button_->setText(isFullScreen() ? "Window" : "Full screen");
    }
  }

private:
  QWidget * makeHeader()
  {
    auto * frame = new QFrame;
    frame->setObjectName("header");
    frame->setFixedHeight(52);
    auto * layout = new QHBoxLayout(frame);
    layout->setContentsMargins(24, 0, 24, 0);
    layout->setSpacing(20);
    layout->addWidget(label("1811", "brand"));
    layout->addWidget(label("Vehicle", "headerSection"));
    layout->addStretch();
    connection_ = label(preview_ ? "Simulation" : "Registration offline", "connection");
    layout->addWidget(connection_);
    auto * clock = label("", "clock");
    auto update_clock = [clock]() { clock->setText(QTime::currentTime().toString("h:mm AP")); };
    update_clock();
    auto * clock_timer = new QTimer(frame);
    connect(clock_timer, &QTimer::timeout, frame, update_clock);
    clock_timer->start(10000);
    layout->addWidget(clock);
    return frame;
  }

  QWidget * makeNavigation()
  {
    auto * nav = new QFrame;
    nav->setObjectName("navigation");
    nav->setFixedHeight(76);
    auto * layout = new QHBoxLayout(nav);
    layout->setContentsMargins(20, 8, 20, 8);
    layout->setSpacing(6);
    auto * overview = new QPushButton("Surround view");
    auto * calibrate = new QPushButton("Calibration");
    overview->setIcon(navigationIcon(false));
    calibrate->setIcon(navigationIcon(true));
    overview->setIconSize(QSize(22, 22));
    calibrate->setIconSize(QSize(22, 22));
    overview_nav_ = overview;
    calibration_nav_ = calibrate;
    overview->setObjectName("navButton");
    calibrate->setObjectName("navButton");
    overview->setCheckable(true);
    calibrate->setCheckable(true);
    overview->setChecked(true);
    connect(overview, &QPushButton::clicked, this, [this, overview, calibrate]() {
      overview->setChecked(true);
      calibrate->setChecked(false);
      if (!reviewing_.isEmpty()) {
        call("release", reviewing_);
        reviewing_.clear();
      }
      review_frame_->hide();
      drawer_area_->hide();
    });
    connect(calibrate, &QPushButton::clicked, this, [this, overview, calibrate]() {
      overview->setChecked(false);
      calibrate->setChecked(true);
      drawer_area_->show();
    });
    layout->addWidget(overview);
    layout->addWidget(calibrate);
    layout->addStretch();
    fullscreen_button_ = new QPushButton("Full screen");
    fullscreen_button_->setObjectName("windowButton");
    fullscreen_button_->setToolTip("Toggle full screen (F11)");
    connect(fullscreen_button_, &QPushButton::clicked, this, [this]() { toggleFullscreen(); });
    layout->addWidget(fullscreen_button_);
    return nav;
  }

  void toggleFullscreen()
  {
    if (isFullScreen()) {
      showNormal();
      fullscreen_button_->setText("Full screen");
    } else {
      showFullScreen();
      fullscreen_button_->setText("Window");
    }
  }

  QWidget * makeLegend()
  {
    auto * panel = new QFrame;
    panel->setObjectName("legend");
    panel->setFixedHeight(58);
    auto * layout = new QHBoxLayout(panel);
    layout->setContentsMargins(12, 5, 12, 5);
    layout->setSpacing(4);
    addLayer(layout, "LiDAR", "#d2e0f0", "LiDAR");
    for (const auto & camera : kCameras) {
      const QString text = QString(camera.key).left(1).toUpper() + QString(camera.key).mid(1);
      addLayer(layout, text, camera.color, camera.display);
    }
    layout->addStretch();
    return panel;
  }

  void addLayer(QHBoxLayout * layout, const QString & text, const QString & color, const QString & display)
  {
    auto * button = new QToolButton;
    button->setCheckable(true);
    button->setChecked(true);
    button->setText("●  " + text);
    button->setToolTip("Toggle " + text + " point cloud");
    button->setMinimumHeight(44);
    layer_buttons_[display] = button;
    button->setStyleSheet(QString(
      "QToolButton { color:%1; background:#292929; border:0; "
      "border-radius:5px; padding:7px 10px; font-size:12px; font-weight:600; }"
      "QToolButton:!checked { color:#777777; background:transparent; }").arg(color));
    connect(button, &QToolButton::toggled, this, [this, display](bool enabled) {
      if (preview_scene_) {
        preview_scene_->setLayerVisible(display, enabled);
      }
      if (!rviz_ || !rviz_->getManager()) {
        return;
      }
      auto * group = rviz_->getManager()->getRootDisplayGroup();
      for (int index = 0; index < group->numDisplays(); ++index) {
        auto * item = group->getDisplayAt(index);
        if (item && item->getName() == display) {
          item->setEnabled(enabled);
          break;
        }
      }
    });
    layout->addWidget(button);
  }

  QFrame * makeCalibrationDrawer()
  {
    auto * panel = new QFrame;
    panel->setObjectName("drawer");
    auto * layout = new QVBoxLayout(panel);
    layout->setContentsMargins(28, 24, 28, 24);
    layout->setSpacing(14);
    auto * title_row = new QHBoxLayout;
    title_row->addWidget(label("Calibration", "drawerTitle"));
    title_row->addStretch();
    auto * close_panel = new QPushButton("×");
    close_panel->setObjectName("closeButton");
    close_panel->setFixedSize(44, 44);
    close_panel->setAccessibleName("Close calibration");
    connect(close_panel, &QPushButton::clicked, this, [this]() { overview_nav_->click(); });
    title_row->addWidget(close_panel);
    layout->addLayout(title_row);
    auto * description = label("Align each camera with the LiDAR. Keep the vehicle stationary.", "bodyText");
    description->setWordWrap(true);
    layout->addWidget(description);
    begin_ = new QPushButton("Begin calibration");
    begin_->setObjectName("primaryButton");
    begin_->setMinimumHeight(52);
    begin_->setEnabled(preview_);
    connect(begin_, &QPushButton::clicked, this, [this]() { call("start", ""); });
    layout->addWidget(begin_);
    layout->addSpacing(8);
    layout->addWidget(label("Cameras", "sectionTitle"));
    auto * grid = new QGridLayout;
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(10);
    for (std::size_t index = 0; index < kCameras.size(); ++index) {
      const auto & camera = kCameras[index];
      const QString title = QString(camera.key).left(1).toUpper() + QString(camera.key).mid(1);
      auto * button = new CameraButton(title, QColor(camera.color));
      button->setObjectName(QString("camera_%1").arg(camera.key));
      button->setEnabled(false);
      camera_buttons_[camera.key] = button;
      connect(button, &QPushButton::clicked, this, [this, name = QString(camera.key)]() { cameraClicked(name); });
      grid->addWidget(button, static_cast<int>(index / 2), static_cast<int>(index % 2));
    }
    layout->addLayout(grid);
    details_ = label("", "hint");
    details_->setWordWrap(true);
    details_->hide();
    layout->addWidget(details_);
    review_frame_ = new QFrame;
    review_frame_->setObjectName("reviewFrame");
    auto * review_layout = new QVBoxLayout(review_frame_);
    review_layout->setContentsMargins(14, 14, 14, 14);
    review_layout->setSpacing(8);
    review_title_ = label("Review camera", "reviewTitle");
    review_message_ = label("Waiting for the selected preview...", "bodyText");
    review_message_->setWordWrap(true);
    review_layout->addWidget(review_title_);
    review_layout->addWidget(review_message_);
    accept_ = new QPushButton("Accept calibration");
    accept_->setObjectName("primaryButton");
    accept_->setMinimumHeight(48);
    accept_->setEnabled(false);
    auto * retry = new QPushButton("Decline and recompute");
    retry->setObjectName("secondaryButton");
    retry->setMinimumHeight(48);
    auto * cancel = new QPushButton("Close review");
    cancel->setObjectName("textButton");
    connect(accept_, &QPushButton::clicked, this, [this]() {
      const QString camera = reviewing_;
      call("accept", camera, [this](bool ok, const QString &) {
        if (ok) { reviewing_.clear(); review_frame_->hide(); }
      });
    });
    connect(retry, &QPushButton::clicked, this, [this]() {
      const QString camera = reviewing_;
      call("recompute", camera, [this](bool ok, const QString &) {
        if (ok) { reviewing_.clear(); review_frame_->hide(); }
      });
    });
    connect(cancel, &QPushButton::clicked, this, [this]() {
      call("release", reviewing_);
      reviewing_.clear();
      review_frame_->hide();
    });
    review_layout->addWidget(accept_);
    review_layout->addWidget(retry);
    review_layout->addWidget(cancel);
    review_frame_->hide();
    layout->addWidget(review_frame_);
    message_ = label("", "message");
    message_->setWordWrap(true);
    layout->addWidget(message_);
    layout->addStretch();
    auto * foot = label("Camera clouds appear after an alignment estimate is available.", "hint");
    foot->setWordWrap(true);
    layout->addWidget(foot);
    return panel;
  }

  void cameraClicked(const QString & name)
  {
    const auto state = camera_status_.value(name).toObject().value("state").toString();
    if (state == "accepted") {
      call("recompute", name);
      return;
    }
    if (state != "ready" && state != "reviewing") {
      return;
    }
    for (const auto & camera : kCameras) {
      if (name == camera.key) {
        layer_buttons_.at(camera.display)->setChecked(true);
        break;
      }
    }
    if (!reviewing_.isEmpty() && reviewing_ != name) {
      call("release", reviewing_);
      reviewing_.clear();
    }
    if (state == "reviewing") {
      reviewing_ = name;
      review_title_->setText("Review " + name + " camera");
      showReview();
      return;
    }
    call("review", name, [this, name](bool ok, const QString &) {
      if (ok) {
        reviewing_ = name;
        review_title_->setText("Review " + name + " camera");
        review_message_->setText("Waiting for the selected preview. Other estimates continue in the background.");
        accept_->setEnabled(false);
        showReview();
      }
    });
  }

  void showReview()
  {
    review_frame_->show();
    QTimer::singleShot(0, drawer_area_, [this]() {
      if (review_frame_->isVisible()) {
        drawer_area_->ensureWidgetVisible(review_frame_, 0, 16);
      }
    });
  }

  using ResultHandler = std::function<void(bool, const QString &)>;

  void call(const QString & action, const QString & camera, ResultHandler done = {})
  {
    if (preview_) {
      message_->setText("Layout preview only. Connect to the vehicle to use calibration controls.");
      if (done) { done(false, "Preview only"); }
      return;
    }
    if (!node_) {
      return;
    }
    const std::string service = std::string(kHubPrefix) +
      (action == "start" ? "/start" : "/" + camera.toStdString() + "/" + action.toStdString());
    auto & client = clients_[service];
    if (!client) {
      client = node_->create_client<std_srvs::srv::Trigger>(service);
    }
    if (!client->service_is_ready()) {
      message_->setText("Calibration service unavailable. Start the stack with hub:=true.");
      if (done) { done(false, "Service unavailable"); }
      return;
    }
    message_->setText("Sending request...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request,
      [this, done](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool ok = false;
        QString text;
        try {
          const auto result = future.get();
          ok = result->success;
          text = QString::fromStdString(result->message);
        } catch (const std::exception & error) {
          text = QString::fromStdString(error.what());
        }
        QMetaObject::invokeMethod(this, [this, done, ok, text]() {
          message_->setText(text);
          if (done) { done(ok, text); }
        }, Qt::QueuedConnection);
      });
  }

  void receiveStatus(const QString & data)
  {
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(data.toUtf8(), &error);
    if (error.error != QJsonParseError::NoError || !document.isObject()) {
      return;
    }
    last_status_.restart();
    const auto status = document.object();
    const bool active = status.value("active").toBool();
    connection_->setText("Registration connected");
    scene_status_->setText(status.value("lidar_online").toBool() ? "LiDAR live" : "Waiting for LiDAR");
    begin_->setEnabled(!active);
    begin_->setText(active ? "Calibration running" : "Begin calibration");
    camera_status_ = status.value("cameras").toObject();
    QStringList diagnostics;
    for (const auto & camera : kCameras) {
      const auto details = camera_status_.value(camera.key).toObject();
      const auto state = details.value("state").toString("idle");
      const bool online = details.value("sensor_online").toBool();
      QString summary;
      if (state == "accepted") {
        summary = "Accepted · recalibrate";
      } else if (state == "applying") {
        summary = "Applying calibration";
      } else if (state == "reviewing") {
        summary = "Reviewing";
      } else if (state == "ready") {
        const int count = details.value("confirmations").toInt();
        const int required = details.value("required").toInt();
        summary = online ? (count > 0 ? QString("Ready · %1 / %2").arg(count).arg(required) :
          "Ready for review") : "Camera offline";
      } else if (state == "searching") {
        summary = QString("Searching · %1 / %2").arg(details.value("confirmations").toInt()).arg(
          details.value("required").toInt());
      } else {
        summary = "Waiting to start";
      }
      auto * button = camera_buttons_.at(camera.key);
      button->setSummary(summary);
      button->setToolTip(details.value("detail").toString());
      button->setEnabled(state == "accepted" || state == "reviewing" || (state == "ready" && online));
      const QString detail = details.value("detail").toString();
      if ((state == "searching" || state == "applying") && !detail.isEmpty()) {
        diagnostics.append(QString("%1 · %2").arg(camera.title, detail));
      }
      if (detail.contains("update failed", Qt::CaseInsensitive) || detail.startsWith("Could not apply") ||
        detail.contains("could not save", Qt::CaseInsensitive))
      {
        message_->setText(QString("%1: %2").arg(camera.title, detail));
      }
    }
    details_->setText(diagnostics.join("\n"));
    details_->setVisible(!diagnostics.isEmpty());
    if (!reviewing_.isEmpty()) {
      const auto current = camera_status_.value(reviewing_).toObject();
      const bool visible = current.value("review_visible").toBool() && current.value("sensor_online").toBool();
      accept_->setEnabled(visible && current.value("state").toString() == "reviewing");
      review_message_->setText(visible ?
        "Review this fixed pose in the 3D view. Estimates continue in the background." :
        "Waiting for a fresh camera preview before acceptance.");
    }
  }

  void populatePreview()
  {
    QJsonObject cameras;
    for (const auto & camera : kCameras) {
      QJsonObject details;
      details["state"] = QString(camera.key) == "left" ? "searching" : "ready";
      details["sensor_online"] = true;
      details["confirmations"] = QString(camera.key) == "left" ? 1 : 3;
      details["required"] = 3;
      cameras[camera.key] = details;
    }
    QJsonObject status;
    status["active"] = true;
    status["lidar_online"] = true;
    status["cameras"] = cameras;
    receiveStatus(QString::fromUtf8(QJsonDocument(status).toJson(QJsonDocument::Compact)));
    connection_->setText("Simulation");
    scene_status_->setText("Simulation");
  }

  static QString styles()
  {
    return R"(
      QWidget { font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; color: #eeeeee; font-size: 13px; }
      QWidget#root { background: #111111; }
      QFrame#header { background: #101010; border-bottom: 1px solid #2c2c2c; }
      QLabel#brand { color: #f4f4f4; font-size: 16px; font-weight: 700; letter-spacing: 1px; }
      QLabel#headerSection, QLabel#connection { color: #a7a7a7; font-size: 12px; }
      QLabel#clock { color: #eeeeee; font-size: 13px; font-weight: 600; }
      QFrame#sceneBar { background: #161616; }
      QLabel#sceneTitle { color: #eeeeee; font-size: 16px; font-weight: 600; }
      QLabel#sceneStatus { color: #a7a7a7; font-size: 12px; }
      QFrame#viewFrame { background: #111111; border: 0; }
      QFrame#legend { background: #161616; border-top: 1px solid #303030; }
      QLabel#sectionCaption { color: #a7a7a7; font-size: 12px; }
      QLabel#hint { color: #999999; font-size: 12px; }
      QFrame#navigation { background: #0c0c0c; border-top: 1px solid #303030; }
      QPushButton#navButton { color: #bfbfbf; background: transparent; border: 0; border-radius: 6px; min-width: 150px; min-height: 52px; padding: 0 16px; font-size: 13px; font-weight: 600; }
      QPushButton#navButton:hover { background: #292929; }
      QPushButton#navButton:checked { color: #ffffff; background: #393939; }
      QPushButton#windowButton { color: #c4c4c4; background: transparent; border: 1px solid #4a4a4a; border-radius: 5px; min-height: 42px; padding: 0 16px; }
      QPushButton#windowButton:hover { background: #303030; }
      QFrame#drawer { background: #1c1c1c; }
      QScrollArea#drawerArea { background: #1c1c1c; border: 0; border-left: 1px solid #383838; }
      QLabel#drawerTitle { color: #f5f5f5; font-size: 25px; font-weight: 600; }
      QLabel#sectionTitle { color: #eeeeee; font-size: 15px; font-weight: 600; }
      QLabel#bodyText { color: #b2b2b2; font-size: 13px; }
      QPushButton#closeButton { color: #c4c4c4; background: transparent; border: 0; border-radius: 5px; font-size: 26px; }
      QPushButton#closeButton:hover { background: #383838; }
      QPushButton#primaryButton { background: #286dda; border: 0; border-radius: 5px; color: #ffffff; padding: 11px; font-weight: 600; }
      QPushButton#primaryButton:hover { background: #3580ee; }
      QPushButton#primaryButton:disabled { background: #373737; color: #a0a0a0; }
      QPushButton#secondaryButton { background: #303030; border: 1px solid #505050; border-radius: 5px; color: #eeeeee; padding: 10px; font-weight: 600; }
      QPushButton#secondaryButton:hover { background: #3b3b3b; }
      QPushButton#textButton { border: 0; color: #bdbdbd; min-height: 40px; }
      QFrame#reviewFrame { background: #272727; border: 1px solid #484848; border-radius: 5px; }
      QLabel#reviewTitle { color: #f0f0f0; font-size: 15px; font-weight: 600; }
      QLabel#message { color: #edbd8a; font-size: 12px; }
      QScrollBar:vertical { background: #1c1c1c; width: 10px; margin: 0; }
      QScrollBar::handle:vertical { background: #565656; min-height: 30px; border-radius: 5px; }
      QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
    )";
  }

  rviz_common::VisualizationFrame * rviz_ = nullptr;
  PreviewScene * preview_scene_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  bool preview_ = false;
  QElapsedTimer last_status_;
  QLabel * connection_ = nullptr;
  QLabel * scene_status_ = nullptr;
  QLabel * message_ = nullptr;
  QLabel * details_ = nullptr;
  QLabel * review_title_ = nullptr;
  QLabel * review_message_ = nullptr;
  QPushButton * begin_ = nullptr;
  QPushButton * overview_nav_ = nullptr;
  QPushButton * calibration_nav_ = nullptr;
  QPushButton * fullscreen_button_ = nullptr;
  QPushButton * accept_ = nullptr;
  QFrame * drawer_ = nullptr;
  QScrollArea * drawer_area_ = nullptr;
  QFrame * review_frame_ = nullptr;
  QString reviewing_;
  QJsonObject camera_status_;
  std::map<std::string, CameraButton *> camera_buttons_;
  std::map<QString, QToolButton *> layer_buttons_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  const bool preview = std::any_of(argv + 1, argv + argc, [](const char * value) {
    return std::string(value) == "--preview";
  });
  if (!preview) {
    rclcpp::init(argc, argv);
  }
  QApplication app(argc, argv);
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node;
  rclcpp::Node::SharedPtr control_node;
  rviz_common::VisualizationFrame * rviz = nullptr;
  QWidget * viewport = nullptr;
  if (preview) {
    viewport = new PreviewScene;
  } else {
    rviz_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("vehicle_hub_rviz");
    control_node = rclcpp::Node::make_shared("vehicle_hub_controls");
    rviz = new rviz_common::VisualizationFrame(rviz_node);
    rviz->setApp(&app);
    rviz->setSplashPath("");
    const auto config = ament_index_cpp::get_package_share_directory("vehicle_ui_hub") + "/config/hub.rviz";
    rviz->initialize(rviz_node, QString::fromStdString(config));
    rviz->setHideButtonVisibility(false);
    rviz->menuBar()->hide();
    rviz->statusBar()->hide();
    for (auto * toolbar : rviz->findChildren<QToolBar *>()) { toolbar->hide(); }
    for (auto * dock : rviz->findChildren<QDockWidget *>()) { dock->hide(); }
    viewport = rviz;
  }
  auto window = std::make_unique<HubWindow>(viewport, rviz, control_node, preview);
  const QStringList args = app.arguments();
  if (args.contains("--windowed")) {
    window->show();
  } else {
    window->showFullScreen();
  }
  if (preview && args.contains("--calibration")) {
    window->showCalibration();
  }
  if (preview && args.contains("--orbit-preview")) {
    QTimer::singleShot(100, viewport, [viewport]() {
      QMouseEvent press(QEvent::MouseButtonPress, QPointF(200, 200), QPointF(200, 200),
        Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
      QMouseEvent move(QEvent::MouseMove, QPointF(300, 250), QPointF(300, 250),
        Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
      QMouseEvent release(QEvent::MouseButtonRelease, QPointF(300, 250), QPointF(300, 250),
        Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
      QApplication::sendEvent(viewport, &press);
      QApplication::sendEvent(viewport, &move);
      QApplication::sendEvent(viewport, &release);
    });
  }
  const int snapshot = args.indexOf("--snapshot");
  if (preview && snapshot >= 0 && snapshot + 1 < args.size()) {
    const QString path = args[snapshot + 1];
    QTimer::singleShot(300, window.get(), [&window, &app, path]() {
      window->grab().save(path);
      app.quit();
    });
  }
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread ros_thread;
  if (!preview) {
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    // RViz owns and spins its visualization node; only spin our controls here.
    executor->add_node(control_node);
    ros_thread = std::thread([&executor]() { executor->spin(); });
  }
  const int result = app.exec();
  if (executor) {
    executor->cancel();
    ros_thread.join();
    window.reset();
    executor.reset();
    control_node.reset();
    rviz_node.reset();
    rclcpp::shutdown();
  }
  return result;
}
