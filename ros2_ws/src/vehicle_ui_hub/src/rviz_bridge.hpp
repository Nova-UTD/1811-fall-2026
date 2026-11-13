// Own the native RViz viewport and expose the small operations the hub needs.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <QObject>
#include <QPointF>
#include <QVector>

class QWidget;
class QString;
class QTouchEvent;
class QWindow;

namespace vehicle_ui_hub
{
// Filter RViz's native render window to keep orbit and pinch zoom centered.
class ViewportControls : public QObject
{
public:
  static constexpr float kMaxDistance = 18.0F;
  using Rotate = std::function<void(const QPointF &)>;
  using Zoom = std::function<void(double)>;
  ViewportControls(Rotate rotate, Zoom zoom, QObject * parent = nullptr);
  void attach(QWidget * widget);
  void attach(QWindow * window);

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  struct Contact { int id; QPointF position; };
  void handleTouch(QTouchEvent * event);
  Rotate rotate_;
  Zoom zoom_;
  QVector<Contact> previous_touch_;
  bool mouse_orbit_ = false;
};

class RvizBridge
{
public:
  explicit RvizBridge(const std::string & config_path);
  ~RvizBridge();
  RvizBridge(const RvizBridge &) = delete;
  RvizBridge & operator=(const RvizBridge &) = delete;

  QWidget * widget() const;
  void queueRender();
  std::uint64_t frameCount() const;
  bool setDisplayEnabled(const QString & name, bool enabled);
  bool pause(bool restore_render_on_failure);
  bool resume(bool start_render);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

// The Python side uses these functions through ctypes. All calls must run on
// the Qt GUI thread, and the returned widget belongs to the bridge until it is
// deleted by its Qt parent or by nova_rviz_destroy().
extern "C" {
int nova_rviz_qt_major();
const char * nova_rviz_error();
void * nova_rviz_create(const char * config_path, int argc, const char * const * argv);
void * nova_rviz_widget(void * bridge);
int nova_rviz_queue_render(void * bridge);
std::uint64_t nova_rviz_frame_count(void * bridge);
int nova_rviz_set_display(void * bridge, const char * name, int enabled);
int nova_rviz_pause(void * bridge, int restore_render_on_failure);
int nova_rviz_resume(void * bridge, int start_render);
void nova_rviz_destroy(void * bridge);
}
