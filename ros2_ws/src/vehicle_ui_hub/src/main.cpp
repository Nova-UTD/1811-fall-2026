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
#include <QDockWidget>
#include <QElapsedTimer>
#include <QFrame>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
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
      QColor color = colors[i % colors.size()];
      color.setAlpha(120 + (i % 120));
      points_.push_back({point, color});
    }
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QLinearGradient background(0, 0, width(), height());
    background.setColorAt(0, QColor("#101b29"));
    background.setColorAt(1, QColor("#080e17"));
    p.fillRect(rect(), background);
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
    p.setPen(QPen(QColor(60, 83, 100, 130), 1));
    for (int index = -6; index <= 6; ++index) {
      p.drawLine(project(QVector3D(index, -6, 0)), project(QVector3D(index, 6, 0)));
      p.drawLine(project(QVector3D(-6, index, 0)), project(QVector3D(6, index, 0)));
    }
    for (const auto & point : points_) {
      p.setPen(QPen(point.second, 1.5));
      p.drawPoint(project(point.first));
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
    p.setPen(QColor("#8296a8"));
    p.drawText(QRect(24, height() - 40, width() - 48, 20),
      Qt::AlignLeft | Qt::AlignVCenter, "LAYOUT PREVIEW  /  SIMULATED POINTS");
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
  std::vector<std::pair<QVector3D, QColor>> points_;
  QPoint last_mouse_;
  float yaw_ = 0.9F;
  float pitch_ = 0.60F;
  float distance_ = 12.0F;
};

class HubWindow : public QMainWindow
{
public:
  HubWindow(QWidget * viewport, rviz_common::VisualizationFrame * rviz,
    rclcpp::Node::SharedPtr node, bool preview)
  : rviz_(rviz), node_(std::move(node)), preview_(preview)
  {
    setWindowTitle("1811 Vehicle Hub");
    resize(1440, 900);
    setMinimumSize(960, 640);
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
    body_layout->addWidget(makeNavigation());

    auto * main = new QWidget;
    auto * main_layout = new QVBoxLayout(main);
    main_layout->setContentsMargins(26, 22, 26, 22);
    main_layout->setSpacing(14);
    auto * title_row = new QHBoxLayout;
    auto * title_stack = new QVBoxLayout;
    title_stack->setSpacing(3);
    title_stack->addWidget(label("PERCEPTION", "eyebrow"));
    title_stack->addWidget(label("Live sensor view", "pageTitle"));
    title_stack->addWidget(label("ZED clouds appear when an alignment estimate is available.", "hint"));
    title_row->addLayout(title_stack);
    title_row->addStretch();
    scene_status_ = label(preview_ ? "DEMO MODE" : "WAITING FOR SENSORS", "sceneStatus");
    title_row->addWidget(scene_status_);
    main_layout->addLayout(title_row);

    auto * view_frame = new QFrame;
    view_frame->setObjectName("viewFrame");
    auto * view_layout = new QVBoxLayout(view_frame);
    view_layout->setContentsMargins(1, 1, 1, 1);
    view_layout->addWidget(viewport);
    main_layout->addWidget(view_frame, 1);
    main_layout->addWidget(makeLegend());
    body_layout->addWidget(main, 1);
    drawer_ = makeCalibrationDrawer();
    drawer_area_ = new QScrollArea;
    drawer_area_->setObjectName("drawerArea");
    drawer_area_->setFixedWidth(350);
    drawer_area_->setWidgetResizable(true);
    drawer_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    drawer_area_->setWidget(drawer_);
    drawer_area_->hide();
    body_layout->addWidget(drawer_area_);
    outer->addWidget(body, 1);
    setCentralWidget(root);

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
          connection_->setText("●  WAITING FOR REGISTRATION");
          scene_status_->setText("WAITING FOR SENSORS");
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

private:
  QWidget * makeHeader()
  {
    auto * frame = new QFrame;
    frame->setObjectName("header");
    frame->setFixedHeight(74);
    auto * layout = new QHBoxLayout(frame);
    layout->setContentsMargins(26, 0, 28, 0);
    layout->setSpacing(13);
    auto * emblem = label("N", "emblem");
    emblem->setAlignment(Qt::AlignCenter);
    emblem->setFixedSize(38, 38);
    layout->addWidget(emblem);
    auto * brand = new QVBoxLayout;
    brand->setSpacing(0);
    brand->setAlignment(Qt::AlignVCenter);
    brand->addWidget(label("NOVA  /  1811", "brand"));
    brand->addWidget(label("VEHICLE INTERFACE", "brandSub"));
    layout->addLayout(brand);
    layout->addStretch();
    connection_ = label(preview_ ? "●  LAYOUT PREVIEW" : "●  WAITING FOR REGISTRATION", "connection");
    connection_->setFixedHeight(32);
    layout->addWidget(connection_);
    return frame;
  }

  QWidget * makeNavigation()
  {
    auto * nav = new QFrame;
    nav->setObjectName("navigation");
    nav->setFixedWidth(176);
    auto * layout = new QVBoxLayout(nav);
    layout->setContentsMargins(14, 28, 14, 22);
    layout->setSpacing(8);
    layout->addWidget(label("WORKSPACE", "navCaption"));
    auto * overview = new QPushButton("◉   Overview");
    auto * calibrate = new QPushButton("◎   Calibration");
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
    layout->addWidget(label("ROS 2  /  VEHICLE", "navFoot"));
    return nav;
  }

  QWidget * makeLegend()
  {
    auto * panel = new QFrame;
    panel->setObjectName("legend");
    auto * layout = new QHBoxLayout(panel);
    layout->setContentsMargins(16, 11, 16, 11);
    layout->setSpacing(8);
    layout->addWidget(label("LAYERS", "eyebrow"));
    layout->addSpacing(12);
    addLayer(layout, "LiDAR", "#d2e0f0", "LiDAR");
    for (const auto & camera : kCameras) {
      addLayer(layout, camera.title, camera.color, camera.display);
    }
    layout->addStretch();
    layout->addWidget(label("DRAG TO ORBIT  ·  SCROLL TO ZOOM", "hint"));
    return panel;
  }

  void addLayer(QHBoxLayout * layout, const QString & text, const QString & color, const QString & display)
  {
    auto * button = new QToolButton;
    button->setCheckable(true);
    button->setChecked(true);
    button->setText("●  " + text);
    layer_buttons_[display] = button;
    button->setStyleSheet(QString(
      "QToolButton { color:%1; background:#172432; border:1px solid #25394a; "
      "border-radius:11px; padding:7px 10px; font-size:11px; font-weight:600; }"
      "QToolButton:!checked { color:#5a6b79; background:#111a25; border-color:#202d3a; }").arg(color));
    connect(button, &QToolButton::toggled, this, [this, display](bool enabled) {
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
    layout->setContentsMargins(24, 28, 24, 24);
    layout->setSpacing(11);
    layout->addWidget(label("CALIBRATION", "eyebrow"));
    layout->addWidget(label("Camera alignment", "drawerTitle"));
    auto * description = label("Review each camera against the live LiDAR view. Estimates keep updating until accepted.", "bodyText");
    description->setWordWrap(true);
    layout->addWidget(description);
    layout->addSpacing(7);
    begin_ = new QPushButton("Begin calibration");
    begin_->setObjectName("primaryButton");
    begin_->setEnabled(preview_);
    connect(begin_, &QPushButton::clicked, this, [this]() { call("start", ""); });
    layout->addWidget(begin_);
    layout->addSpacing(12);
    layout->addWidget(label("CAMERAS", "sectionCaption"));
    for (const auto & camera : kCameras) {
      auto * button = new QPushButton;
      button->setMinimumHeight(66);
      button->setEnabled(false);
      button->setStyleSheet(QString(
        "QPushButton { text-align:left; padding:11px 13px; color:#dbe6ef; "
        "background:#152230; border:1px solid #263a49; border-left:3px solid %1; "
        "border-radius:7px; font-size:12px; font-weight:600; }"
        "QPushButton:hover { background:#1d3040; }"
        "QPushButton:disabled { color:#718392; background:#121e29; }").arg(camera.color));
      button->setText(QString("%1 CAMERA\nWaiting for calibration").arg(camera.title));
      camera_buttons_[camera.key] = button;
      connect(button, &QPushButton::clicked, this, [this, name = QString(camera.key)]() { cameraClicked(name); });
      layout->addWidget(button);
    }
    details_ = label("", "hint");
    details_->setWordWrap(true);
    details_->hide();
    layout->addWidget(details_);
    review_frame_ = new QFrame;
    review_frame_->setObjectName("reviewFrame");
    auto * review_layout = new QVBoxLayout(review_frame_);
    review_layout->setContentsMargins(14, 14, 14, 14);
    review_layout->setSpacing(8);
    review_title_ = label("REVIEW", "reviewTitle");
    review_message_ = label("Waiting for the selected preview...", "bodyText");
    review_message_->setWordWrap(true);
    review_layout->addWidget(review_title_);
    review_layout->addWidget(review_message_);
    accept_ = new QPushButton("Accept calibration");
    accept_->setObjectName("primaryButton");
    accept_->setEnabled(false);
    auto * retry = new QPushButton("Decline and recompute");
    retry->setObjectName("secondaryButton");
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
    auto * foot = label("Keep the vehicle stationary while calibrating.", "hint");
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
      review_title_->setText(name.toUpper() + " CAMERA  /  REVIEW");
      review_frame_->show();
      return;
    }
    call("review", name, [this, name](bool ok, const QString &) {
      if (ok) {
        reviewing_ = name;
        review_title_->setText(name.toUpper() + " CAMERA  /  REVIEW");
        review_message_->setText("Waiting for the selected preview. Other estimates continue in the background.");
        accept_->setEnabled(false);
        review_frame_->show();
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
    connection_->setText("●  CALIBRATION CONNECTED");
    scene_status_->setText(status.value("lidar_online").toBool() ? "LIDAR LIVE" : "WAITING FOR LIDAR");
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
        summary = "Accepted  ·  tap to recalibrate";
      } else if (state == "applying") {
        summary = "Applying accepted calibration";
      } else if (state == "reviewing") {
        summary = "Review in progress";
      } else if (state == "ready") {
        const int count = details.value("confirmations").toInt();
        const int required = details.value("required").toInt();
        summary = online ? (count > 0 ? QString("Ready to review  ·  %1 / %2").arg(count).arg(required) :
          "Ready for review") : "Waiting for camera stream";
      } else if (state == "searching") {
        summary = QString("Calibrating  %1 / %2").arg(details.value("confirmations").toInt()).arg(
          details.value("required").toInt());
      } else {
        summary = "Waiting for calibration";
      }
      auto * button = camera_buttons_.at(camera.key);
      button->setText(QString("%1 CAMERA\n%2").arg(camera.title, summary));
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
    connection_->setText("●  LAYOUT PREVIEW");
    scene_status_->setText("DEMO MODE");
  }

  static QString styles()
  {
    return R"(
      QWidget { font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; color: #dce7ef; font-size: 13px; }
      QWidget#root { background: #0b121b; }
      QFrame#header { background: #101a25; border-bottom: 1px solid #23313f; }
      QLabel#emblem { background: #253b4a; border: 1px solid #476777; border-radius: 10px; color: #a9e5ed; font-size: 19px; font-weight: 700; }
      QLabel#brand { color: #eef5f8; font-size: 13px; font-weight: 700; letter-spacing: 1px; }
      QLabel#brandSub, QLabel#navFoot { color: #748a9b; font-size: 9px; font-weight: 600; letter-spacing: 1px; }
      QLabel#connection { color: #8ccfc8; background: #172d32; border: 1px solid #294f52; border-radius: 11px; padding: 8px 11px; font-size: 10px; font-weight: 700; }
      QFrame#navigation { background: #0f1924; border-right: 1px solid #22303d; }
      QLabel#navCaption, QLabel#eyebrow, QLabel#sectionCaption { color: #71899b; font-size: 10px; font-weight: 700; letter-spacing: 1px; }
      QPushButton#navButton { text-align: left; color: #91a6b7; border: 0; border-radius: 8px; padding: 14px 12px; font-size: 12px; }
      QPushButton#navButton:hover { background: #192c3b; }
      QPushButton#navButton:checked { color: #e8f7f8; background: #23404a; }
      QLabel#pageTitle { color: #f1f6f9; font-size: 24px; font-weight: 700; }
      QLabel#sceneStatus { color: #89c9c0; background: #14282c; border: 1px solid #26454a; border-radius: 10px; padding: 7px 10px; font-size: 10px; font-weight: 700; }
      QFrame#viewFrame { background: #0d131d; border: 1px solid #273746; border-radius: 8px; }
      QFrame#legend { background: #111d29; border: 1px solid #263746; border-radius: 9px; }
      QLabel#hint { color: #708594; font-size: 10px; font-weight: 600; }
      QFrame#drawer { background: #111d29; border-left: 1px solid #273a48; }
      QScrollArea#drawerArea { background: #111d29; border: 0; }
      QLabel#drawerTitle { color: #f2f7f9; font-size: 21px; font-weight: 700; }
      QLabel#bodyText { color: #9aadb9; font-size: 11px; line-height: 1.35; }
      QPushButton#primaryButton { background: #78d1c7; border: 0; border-radius: 7px; color: #092023; padding: 13px; font-weight: 700; }
      QPushButton#primaryButton:hover { background: #9ae1d9; }
      QPushButton#primaryButton:disabled { background: #294248; color: #7fa4a2; }
      QPushButton#secondaryButton { background: #263847; border: 1px solid #405465; border-radius: 7px; color: #e3edf2; padding: 11px; font-weight: 600; }
      QPushButton#secondaryButton:hover { background: #324959; }
      QPushButton#textButton { border: 0; color: #91a8b6; padding: 5px; }
      QFrame#reviewFrame { background: #172b36; border: 1px solid #31505b; border-radius: 8px; }
      QLabel#reviewTitle { color: #bce9e5; font-size: 10px; font-weight: 700; letter-spacing: 1px; }
      QLabel#message { color: #e5b78a; font-size: 11px; }
    )";
  }

  rviz_common::VisualizationFrame * rviz_ = nullptr;
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
  QPushButton * calibration_nav_ = nullptr;
  QPushButton * accept_ = nullptr;
  QFrame * drawer_ = nullptr;
  QScrollArea * drawer_area_ = nullptr;
  QFrame * review_frame_ = nullptr;
  QString reviewing_;
  QJsonObject camera_status_;
  std::map<std::string, QPushButton *> camera_buttons_;
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
  window->show();
  const QStringList args = app.arguments();
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
