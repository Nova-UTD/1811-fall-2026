// Keep RViz's native Qt widget, render loop, and input handling behind a small
// interface that can also be called from the Python hub through ctypes.
#include "rviz_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <utility>

#include <QApplication>
#include <QDockWidget>
#include <QLineF>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPointer>
#include <QStatusBar>
#include <QThread>
#include <QToolBar>
#include <QTouchEvent>
#include <QWheelEvent>
#include <QWindow>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/transformation/transformation_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

namespace vehicle_ui_hub
{
namespace
{
rviz_common::properties::FloatProperty * orbitProperty(
  rviz_common::VisualizationManager * manager, const QString & name)
{
  auto * views = manager->getViewManager();
  auto * view = views ? views->getCurrent() : nullptr;
  if (!view || view->getClassId() != "rviz_default_plugins/Orbit") { return nullptr; }
  return qobject_cast<rviz_common::properties::FloatProperty *>(view->subProp(name));
}

void configureControls(rviz_common::VisualizationFrame * frame)
{
  auto * manager = frame->getManager();
  auto * panel = manager ? manager->getRenderPanel() : nullptr;
  auto * distance = manager ? orbitProperty(manager, "Distance") : nullptr;
  if (!panel || !panel->getRenderWindow() || !distance ||
    !orbitProperty(manager, "Yaw") || !orbitProperty(manager, "Pitch"))
  {
    std::fprintf(stderr, "[hub] Warning: could not attach custom orbit controls to the RViz viewport.\n");
    return;
  }
  distance->setMax(ViewportControls::kMaxDistance);
  auto * controls = new ViewportControls(
    [manager](const QPointF & delta) {
      auto * yaw = orbitProperty(manager, "Yaw");
      auto * pitch = orbitProperty(manager, "Pitch");
      if (!yaw || !pitch) { return; }
      constexpr float full_turn = 6.28318530718F;
      float angle = std::fmod(yaw->getFloat() - static_cast<float>(delta.x()) * 0.005F, full_turn);
      if (angle < 0.0F) { angle += full_turn; }
      yaw->setFloat(angle);
      pitch->add(static_cast<float>(delta.y()) * 0.005F);
      manager->queueRender();
    },
    [manager](double factor) {
      if (auto * value = orbitProperty(manager, "Distance")) {
        value->multiply(static_cast<float>(factor));
        manager->queueRender();
      }
    }, frame);
  controls->attach(panel);
  controls->attach(panel->getRenderWindow());
  QObject::connect(manager->getViewManager(), &rviz_common::ViewManager::currentChanged, controls, [manager]() {
    if (auto * value = orbitProperty(manager, "Distance")) { value->setMax(ViewportControls::kMaxDistance); }
  });
}
}

class RvizBridge::Impl
{
public:
  ~Impl()
  {
    // The main window may already have deleted its child. QPointer notices that
    // case, so RViz is destroyed exactly once and before its ROS node.
    delete frame.data();
    node.reset();
  }
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node;
  QPointer<rviz_common::VisualizationFrame> frame;
  bool saved_displays_enabled = false;
  std::optional<rviz_common::PluginInfo> saved_transformer;
  std::optional<rviz_common::PluginInfo> idle_transformer;
};

RvizBridge::RvizBridge(const std::string & config_path) : impl_(std::make_unique<Impl>())
{
  auto * app = qobject_cast<QApplication *>(QCoreApplication::instance());
  if (!app || QThread::currentThread() != app->thread()) {
    throw std::runtime_error("RViz must be created on the QApplication thread");
  }
  if (!rclcpp::ok()) { throw std::runtime_error("rclcpp is not initialized"); }
  impl_->node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("vehicle_hub_rviz");
  impl_->frame = new rviz_common::VisualizationFrame(impl_->node);
  impl_->frame->setApp(app);
  impl_->frame->setSplashPath("");
  impl_->frame->initialize(impl_->node, QString::fromStdString(config_path));
  impl_->frame->setHideButtonVisibility(false);
  impl_->frame->menuBar()->hide();
  impl_->frame->statusBar()->hide();
  for (auto * toolbar : impl_->frame->findChildren<QToolBar *>()) { toolbar->hide(); }
  for (auto * dock : impl_->frame->findChildren<QDockWidget *>()) { dock->hide(); }
  configureControls(impl_->frame);
}

RvizBridge::~RvizBridge() = default;
QWidget * RvizBridge::widget() const { return impl_->frame.data(); }

void RvizBridge::queueRender()
{
  if (impl_->frame && impl_->frame->getManager()) { impl_->frame->getManager()->queueRender(); }
}

std::uint64_t RvizBridge::frameCount() const
{
  return impl_->frame && impl_->frame->getManager() ? impl_->frame->getManager()->getFrameCount() : 0;
}

bool RvizBridge::setDisplayEnabled(const QString & name, bool enabled)
{
  auto * manager = impl_->frame ? impl_->frame->getManager() : nullptr;
  auto * group = manager ? manager->getRootDisplayGroup() : nullptr;
  if (!group) { return false; }
  for (int index = 0; index < group->numDisplays(); ++index) {
    auto * display = group->getDisplayAt(index);
    if (display && display->getName() == name) { display->setEnabled(enabled); return true; }
  }
  return false;
}

bool RvizBridge::pause(bool restore_render_on_failure)
{
  auto * manager = impl_->frame ? impl_->frame->getManager() : nullptr;
  if (!manager) { return true; }
  if (impl_->saved_transformer) { return true; }
  auto * transforms = manager->getTransformationManager();
  auto * displays = manager->getRootDisplayGroup();
  if (!transforms || !displays) { return false; }
  const auto choices = transforms->getAvailableTransformers();
  const auto identity = std::find_if(choices.begin(), choices.end(), [](const auto & choice) {
    return choice.id == "rviz_common/Identity";
  });
  if (identity == choices.end()) { return false; }
  impl_->saved_transformer = transforms->getCurrentTransformerInfo();
  impl_->idle_transformer = *identity;
  impl_->saved_displays_enabled = displays->isEnabled();
  manager->stopUpdate();
  bool paused = false;
  try {
    // Preserve individual layer choices while releasing their subscriptions and TF.
    displays->setEnabled(false);
    transforms->setTransformer(*identity);
    paused = transforms->getCurrentTransformerInfo().id == identity->id;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[hub] Warning: could not pause the RViz 3D viewport: %s\n", error.what());
  }
  if (!paused) {
    try { transforms->setTransformer(*impl_->saved_transformer); }
    catch (const std::exception & error) {
      std::fprintf(stderr, "[hub] Warning: could not restore the RViz TF transformer after pause failed: %s\n", error.what());
    }
    displays->setEnabled(impl_->saved_displays_enabled);
    if (restore_render_on_failure) { manager->startUpdate(); }
    impl_->saved_transformer.reset();
    impl_->idle_transformer.reset();
  }
  return paused;
}

bool RvizBridge::resume(bool start_render)
{
  auto * manager = impl_->frame ? impl_->frame->getManager() : nullptr;
  if (!manager) { return true; }
  if (!impl_->saved_transformer) { return true; }
  auto * transforms = manager->getTransformationManager();
  auto * displays = manager->getRootDisplayGroup();
  if (!transforms || !displays || !impl_->saved_transformer || !impl_->idle_transformer) { return false; }
  bool restored = false;
  try {
    transforms->setTransformer(*impl_->saved_transformer);
    restored = transforms->getCurrentTransformerInfo().id == impl_->saved_transformer->id;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[hub] Warning: could not resume the RViz 3D viewport from idle: %s\n", error.what());
  }
  if (!restored) {
    try { transforms->setTransformer(*impl_->idle_transformer); }
    catch (const std::exception & error) {
      std::fprintf(stderr, "[hub] Warning: could not revert RViz to the idle transformer after wake failed: %s\n", error.what());
    }
    return false;
  }
  displays->setEnabled(impl_->saved_displays_enabled);
  impl_->saved_transformer.reset();
  impl_->idle_transformer.reset();
  if (start_render) { manager->startUpdate(); manager->queueRender(); }
  return true;
}
ViewportControls::ViewportControls(Rotate rotate, Zoom zoom, QObject * parent)
: QObject(parent), rotate_(std::move(rotate)), zoom_(std::move(zoom)) {}

void ViewportControls::attach(QWidget * widget)
{
  widget->setAttribute(Qt::WA_AcceptTouchEvents);
  widget->installEventFilter(this);
}

void ViewportControls::attach(QWindow * window)
{
  window->installEventFilter(this);
}

bool ViewportControls::eventFilter(QObject * watched, QEvent * event)
{
  // Let left-drag orbit pass through; block the mouse gestures RViz uses to pan.
  switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
      handleTouch(static_cast<QTouchEvent *>(event));
      event->accept();
      return true;
    case QEvent::TouchEnd:
    case QEvent::TouchCancel:
      previous_touch_.clear();
      event->accept();
      return true;
    case QEvent::MouseButtonPress: {
      const auto * mouse = static_cast<QMouseEvent *>(event);
      if (mouse->button() != Qt::LeftButton) { return true; }
      mouse_orbit_ = !(mouse->buttons() & (Qt::MiddleButton | Qt::RightButton)) &&
        !(mouse->modifiers() & Qt::ShiftModifier);
      return !mouse_orbit_;
    }
    case QEvent::MouseMove: {
      const auto * mouse = static_cast<QMouseEvent *>(event);
      if (mouse->buttons() == Qt::NoButton) { return false; }
      return !mouse_orbit_ || !(mouse->buttons() & Qt::LeftButton) ||
        (mouse->buttons() & (Qt::MiddleButton | Qt::RightButton)) ||
        (mouse->modifiers() & Qt::ShiftModifier);
    }
    case QEvent::MouseButtonRelease: {
      const auto * mouse = static_cast<QMouseEvent *>(event);
      const bool allow_release = mouse_orbit_ && mouse->button() == Qt::LeftButton;
      if (mouse->button() == Qt::LeftButton) { mouse_orbit_ = false; }
      return !allow_release;
    }
    case QEvent::MouseButtonDblClick:
      return true;
    case QEvent::Wheel:
      // RViz normally uses Shift+wheel to move the focal point vertically.
      return static_cast<QWheelEvent *>(event)->modifiers().testFlag(Qt::ShiftModifier);
    default:
      return QObject::eventFilter(watched, event);
  }
}

void ViewportControls::handleTouch(QTouchEvent * event)
{
  if (event->type() == QEvent::TouchBegin) { previous_touch_.clear(); }
  QVector<Contact> current;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  for (const auto & point : event->points()) {
    if (point.state() != QEventPoint::State::Released) {
      // Window-relative coordinates are stable for both widgets and RViz's QWindow.
      current.push_back({point.id(), point.scenePosition()});
    }
  }
#else
  for (const auto & point : event->touchPoints()) {
    if (point.state() != Qt::TouchPointReleased) {
      current.push_back({point.id(), point.pos()});
    }
  }
#endif
  std::sort(current.begin(), current.end(), [](const Contact & a, const Contact & b) { return a.id < b.id; });
  // A change in finger count or identity starts a new baseline and avoids a jump.
  bool same_contacts = current.size() == previous_touch_.size() && current.size() <= 2;
  for (int index = 0; same_contacts && index < current.size(); ++index) {
    same_contacts = current[index].id == previous_touch_[index].id;
  }
  if (same_contacts && current.size() == 1) {
    rotate_(current[0].position - previous_touch_[0].position);
  } else if (same_contacts && current.size() == 2) {
    const double before = QLineF(previous_touch_[0].position, previous_touch_[1].position).length();
    const double after = QLineF(current[0].position, current[1].position).length();
    // A near-zero separation cannot produce a stable scale factor.
    if (before >= 8.0 && after >= 8.0) {
      const double factor = before / after;
      if (std::isfinite(factor)) { zoom_(factor); }
    }
  }
  previous_touch_ = std::move(current);
}
}  // namespace vehicle_ui_hub

namespace
{
thread_local std::string bridge_error;
bool bridge_owns_ros = false;
unsigned bridge_count = 0;

vehicle_ui_hub::RvizBridge * bridgeFrom(void * bridge)
{
  return static_cast<vehicle_ui_hub::RvizBridge *>(bridge);
}

template<typename Function>
int invokeBridge(Function function)
{
  bridge_error.clear();
  try { return function() ? 1 : 0; }
  catch (const std::exception & error) { bridge_error = error.what(); }
  catch (...) { bridge_error = "Unknown RViz error"; }
  return -1;
}
}

extern "C" {
int nova_rviz_qt_major() { return QT_VERSION >> 16; }
const char * nova_rviz_error() { return bridge_error.c_str(); }

void * nova_rviz_create(const char * config_path, int argc, const char * const * argv)
{
  bridge_error.clear();
  try {
    auto * app = qobject_cast<QApplication *>(QCoreApplication::instance());
    if (!app || QThread::currentThread() != app->thread()) {
      throw std::runtime_error("Create QApplication and call RViz on its GUI thread");
    }
    if (!config_path || !*config_path) { throw std::runtime_error("RViz configuration path is empty"); }
    if (!rclcpp::ok()) {
      // rclpy owns application signal handling; RViz needs its own C++ context.
      rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
      bridge_owns_ros = true;
    }
    auto * bridge = new vehicle_ui_hub::RvizBridge(config_path);
    ++bridge_count;
    return bridge;
  } catch (const std::exception & error) {
    bridge_error = error.what();
  } catch (...) {
    bridge_error = "Unknown error creating RViz";
  }
  if (bridge_owns_ros && bridge_count == 0) {
    rclcpp::shutdown();
    bridge_owns_ros = false;
  }
  return nullptr;
}

void * nova_rviz_widget(void * bridge) { return bridge ? bridgeFrom(bridge)->widget() : nullptr; }
int nova_rviz_queue_render(void * bridge)
{
  return invokeBridge([&]() { if (!bridge) { return false; } bridgeFrom(bridge)->queueRender(); return true; });
}
std::uint64_t nova_rviz_frame_count(void * bridge) { return bridge ? bridgeFrom(bridge)->frameCount() : 0; }
int nova_rviz_set_display(void * bridge, const char * name, int enabled)
{
  return invokeBridge([&]() { return bridge && name && bridgeFrom(bridge)->setDisplayEnabled(QString::fromUtf8(name), enabled); });
}
int nova_rviz_pause(void * bridge, int restore_render_on_failure)
{
  return invokeBridge([&]() { return bridge && bridgeFrom(bridge)->pause(restore_render_on_failure); });
}
int nova_rviz_resume(void * bridge, int start_render)
{
  return invokeBridge([&]() { return bridge && bridgeFrom(bridge)->resume(start_render); });
}
void nova_rviz_destroy(void * bridge)
{
  if (!bridge) { return; }
  delete bridgeFrom(bridge);
  if (bridge_count) { --bridge_count; }
  if (bridge_owns_ros && bridge_count == 0) {
    rclcpp::shutdown();
    bridge_owns_ros = false;
  }
}
}
