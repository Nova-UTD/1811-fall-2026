# Custom-painted hub controls and the no-ROS preview scene.
import math
import time

from PyQt5 import QtCore, QtGui, QtWidgets, sip


CAMERAS = (
    ('front', 'FRONT', '#4fc1e6', 'Front ZED'),
    ('left', 'LEFT', '#78d3a7', 'Left ZED'),
    ('rear', 'REAR', '#f0ad69', 'Rear ZED'),
    ('right', 'RIGHT', '#b49ae8', 'Right ZED'),
)

# The preview and native RViz use the same proportions for the calibration
# activity sweep. These are visual sizes in base_link meters, not measurements.
SCAN_INNER_RADIUS = 0.8
SCAN_RADIUS = 5.5
SCAN_WIDTH = 1.1
SCAN_SEGMENTS = 28
SCAN_RINGS = (2.1, 3.8, 5.45)
SCAN_RIM_SEGMENTS = 48


def white_theme():
    app = QtWidgets.QApplication.instance()
    return bool(app and app.property('vehicleHubWhiteTheme'))


def color(black, white):
    return QtGui.QColor(white if white_theme() else black)


def _painter_icon():
    pixmap = QtGui.QPixmap(24, 24)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    painter.setPen(QtGui.QPen(color('#eeeeee', '#1b1d1f'), 1.6, QtCore.Qt.SolidLine,
                              QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
    return pixmap, painter


def navigation_icon(page):
    pixmap, painter = _painter_icon()
    foreground = color('#eeeeee', '#1b1d1f')
    if page == 'idle':
        moon, cutout = QtGui.QPainterPath(), QtGui.QPainterPath()
        moon.addEllipse(QtCore.QRectF(4, 4, 16, 16))
        cutout.addEllipse(QtCore.QRectF(10, 1, 15, 15))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(foreground)
        painter.drawPath(moon.subtracted(cutout))
    elif page == 'sketch':
        for line in ((5, 19, 8, 13), (8, 13, 17, 4), (17, 4, 20, 7),
                     (20, 7, 11, 16), (11, 16, 5, 19), (5, 22, 20, 22)):
            painter.drawLine(*line)
    elif page == 'network':
        painter.drawRoundedRect(QtCore.QRectF(3, 5, 18, 14), 2, 2)
        for x in (7, 11, 15, 19):
            painter.drawLine(x, 5, x, 10)
        painter.drawLine(8, 19, 8, 22)
        painter.drawLine(16, 19, 16, 22)
    elif page == 'tools':
        for x in (4, 13):
            for y in (4, 13):
                painter.drawRoundedRect(QtCore.QRectF(x, y, 7, 7), 1, 1)
    elif page == 'updates':
        painter.drawArc(QtCore.QRectF(4, 4, 16, 16), 35 * 16, 285 * 16)
        painter.drawLine(18, 3, 20, 8)
        painter.drawLine(20, 8, 15, 8)
    elif page == 'recording':
        painter.drawEllipse(QtCore.QPointF(12, 12), 8.5, 8.5)
        painter.setBrush(foreground)
        painter.drawEllipse(QtCore.QPointF(12, 12), 3.5, 3.5)
    elif page == 'calibration':
        for y in (6, 12, 18):
            painter.drawLine(3, y, 21, y)
        painter.setBrush(foreground)
        for x, y in ((8, 6), (16, 12), (10, 18)):
            painter.drawEllipse(QtCore.QPointF(x, y), 2, 2)
    elif page == 'vehicle':
        painter.drawRoundedRect(QtCore.QRectF(3, 9, 18, 8), 2, 2)
        for line in ((6, 9, 8, 5), (8, 5, 16, 5), (16, 5, 18, 9), (6, 17, 6, 20), (18, 17, 18, 20)):
            painter.drawLine(*line)
    else:
        painter.drawEllipse(QtCore.QPointF(12, 12), 8.5, 8.5)
        painter.drawEllipse(QtCore.QPointF(12, 12), 3.5, 3.5)
        painter.drawLine(12, 1, 12, 4)
        painter.drawLine(12, 20, 12, 23)
    painter.end()
    return QtGui.QIcon(pixmap)


def fullscreen_icon(fullscreen):
    pixmap, painter = _painter_icon()
    if fullscreen:
        lines = ((4, 9, 9, 9), (9, 9, 9, 4), (15, 4, 15, 9), (15, 9, 20, 9),
                 (4, 15, 9, 15), (9, 15, 9, 20), (15, 20, 15, 15), (15, 15, 20, 15))
    else:
        lines = ((4, 9, 4, 4), (4, 4, 9, 4), (15, 4, 20, 4), (20, 4, 20, 9),
                 (4, 15, 4, 20), (4, 20, 9, 20), (15, 20, 20, 20), (20, 20, 20, 15))
    for line in lines:
        painter.drawLine(*line)
    painter.end()
    return QtGui.QIcon(pixmap)


def _animation(parent, duration, callback):
    animation = QtCore.QVariantAnimation(parent)
    animation.setDuration(duration)
    animation.setEasingCurve(QtCore.QEasingCurve.OutCubic)
    animation.valueChanged.connect(callback)
    return animation


def _animate(animation, start, end):
    animation.stop()
    animation.setStartValue(start)
    animation.setEndValue(end)
    animation.start()


class AnimatedStatusLabel(QtWidgets.QLabel):
    # Animate short-lived messages, not live measurements. setText updates the
    # real label immediately for accessibility and callers that inspect it.
    def __init__(self, text='', parent=None):
        super().__init__(text, parent)
        self._previous = QtGui.QPixmap()
        self._current = QtGui.QPixmap()
        self._progress, self._capturing = 1.0, False
        self._transition = _animation(self, 280, self._advance)
        self._transition.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._transition.finished.connect(self._finish_transition)
        self._last_change = 0.0
        self._appear_on_show = self._hide_when_empty = False
        self._saved_minimum_height = None

    def _advance(self, progress):
        self._progress = progress
        self.update()

    def _finish_transition(self):
        self._previous = self._current = QtGui.QPixmap()
        self.update()
        self._restore_height()
        if self._hide_when_empty and not self.text():
            self._hide_when_empty = False
            self.hide()

    def setText(self, text):
        if text == self.text():
            return
        now = time.monotonic()
        frequent = now - self._last_change < 0.3
        self._last_change = now
        if not self.isVisible():
            super().setText(text)
            self._appear_on_show = bool(text)
            return
        if self._transition.state() == QtCore.QAbstractAnimation.Running:
            # Keep the current transition's timing but show the newest text.
            if not text and self.text():
                self._preserve_height()
            super().setText(text)
            self._current = self._snapshot()
            self.update()
            return
        if frequent:
            super().setText(text)
            return
        if not text and self.text():
            # A multiline message needs its old height until the fade ends.
            self._preserve_height()
        previous = self._snapshot()
        super().setText(text)
        self._start_transition(previous)

    def _start_transition(self, previous):
        self._previous, self._current = previous, self._snapshot()
        self._progress = 0.0
        _animate(self._transition, 0.0, 1.0)

    def _snapshot(self):
        if not self.text() or self.width() <= 0 or self.height() <= 0:
            return QtGui.QPixmap()
        # Keep only the text on a transparent pixmap. Unlike grab(), this does
        # not copy an opaque background into the transition.
        ratio = self.devicePixelRatioF()
        pixmap = QtGui.QPixmap(self.size() * ratio)
        pixmap.setDevicePixelRatio(ratio)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        self._capturing = True
        try:
            self.render(painter, QtCore.QPoint(), QtGui.QRegion(), QtWidgets.QWidget.DrawChildren)
        finally:
            self._capturing = False
            painter.end()
        return pixmap

    def paintEvent(self, event):
        if self._capturing or self._transition.state() != QtCore.QAbstractAnimation.Running:
            super().paintEvent(event)
            return
        # Compositing the two cached text images directly avoids nested Qt
        # graphics effects when a whole panel is also fading in or out.
        outgoing = min(1.0, self._progress * 2.0)
        incoming = (max(0.0, (self._progress - 0.5) * 2.0)
                    if not self._previous.isNull() else self._progress)
        painter = QtGui.QPainter(self)
        painter.setOpacity(1.0 - outgoing)
        painter.drawPixmap(0, round(-6 * outgoing), self._previous)
        painter.setOpacity(incoming)
        painter.drawPixmap(0, round(6 * (1.0 - incoming)), self._current)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._transition.state() == QtCore.QAbstractAnimation.Running:
            self._current = self._snapshot()

    def set_message(self, text):
        # Optional messages release their layout space only after fading out.
        self._hide_when_empty = False
        self.setText(text)
        if text:
            self.show()
        elif self._transition.state() == QtCore.QAbstractAnimation.Running:
            self._hide_when_empty = True
        else:
            self.hide()

    def showEvent(self, event):
        super().showEvent(event)
        if self._appear_on_show:
            self._appear_on_show = False
            self._start_transition(QtGui.QPixmap())

    def hideEvent(self, event):
        self._transition.stop()
        self._previous = self._current = QtGui.QPixmap()
        self._restore_height()
        super().hideEvent(event)

    def _restore_height(self):
        if self._saved_minimum_height is not None:
            self.setMinimumHeight(self._saved_minimum_height)
            self._saved_minimum_height = None

    def _preserve_height(self):
        if self._saved_minimum_height is None:
            self._saved_minimum_height = self.minimumHeight()
            self.setMinimumHeight(max(self.minimumHeight(), self.height()))


class TouchScrollFilter(QtCore.QObject):
    # A swipe can begin on a panel's labels or buttons, not just its viewport.
    # Short taps still reach the original control. Use pixel scrolling for
    # lists too, so one touchscreen movement cannot jump several rows at once.
    def __init__(self, parent=None):
        super().__init__(parent)
        self._roots = set()
        self._area = self._button = None
        self._areas = []
        self._start = self._last = QtCore.QPoint()
        self._dragging, self._horizontal = False, False
        self._remainder = 0.0
        QtWidgets.qApp.installEventFilter(self)

    def add(self, area):
        self._roots.add(area)

    def _find_area(self, widget):
        areas, button = [], None
        while isinstance(widget, QtWidgets.QWidget):
            if isinstance(widget, (QtWidgets.QAbstractSlider, QtWidgets.QLineEdit, QtWidgets.QComboBox)):
                return [], None
            if widget.objectName().startswith('dragHandle'):
                return [], None
            if button is None and isinstance(widget, QtWidgets.QAbstractButton):
                button = widget
            if isinstance(widget, QtWidgets.QAbstractScrollArea):
                if isinstance(widget, QtWidgets.QAbstractItemView):
                    widget.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
                    widget.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
                vertical, horizontal = widget.verticalScrollBar(), widget.horizontalScrollBar()
                if vertical.maximum() > vertical.minimum() or horizontal.maximum() > horizontal.minimum():
                    areas.append(widget)
            if widget in self._roots:
                return areas, button
            widget = widget.parentWidget()
        return [], None

    def eventFilter(self, watched, event):
        kind = event.type()
        if kind == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            if self._area is None:
                self._areas, self._button = self._find_area(watched)
                self._area = self._areas[0] if self._areas else None
                self._start = self._last = event.globalPos()
                self._dragging = False
                self._remainder = 0.0
        elif kind == QtCore.QEvent.MouseMove and self._area is not None:
            if sip.isdeleted(self._area) or not event.buttons() & QtCore.Qt.LeftButton:
                self._area = self._button = None
                return False
            position = event.globalPos()
            delta = position - self._start
            if not self._dragging and max(abs(delta.x()), abs(delta.y())) >= QtWidgets.QApplication.startDragDistance():
                self._horizontal = abs(delta.x()) > abs(delta.y())
                candidates = [area for area in self._areas if not sip.isdeleted(area)]
                selected = next((area for area in candidates if
                                 (area.horizontalScrollBar() if self._horizontal else area.verticalScrollBar()).maximum() >
                                 (area.horizontalScrollBar() if self._horizontal else area.verticalScrollBar()).minimum()), None)
                if selected:
                    self._area = selected
                    self._dragging = True
                    if self._button and not sip.isdeleted(self._button):
                        self._button.setDown(False)
            if self._dragging:
                bar = self._area.horizontalScrollBar() if self._horizontal else self._area.verticalScrollBar()
                distance = position.x() - self._last.x() if self._horizontal else position.y() - self._last.y()
                # Keep fractional movement between events so slow swipes remain
                # responsive while scrolling at about half the finger speed.
                movement = distance * 0.55 + self._remainder
                whole = int(movement)
                self._remainder = movement - whole
                bar.setValue(bar.value() - whole)
                self._last = position
                event.accept()
                return True
            self._last = position
        elif kind == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.LeftButton and self._area is not None:
            dragging = self._dragging
            self._area = self._button = None
            self._dragging = False
            if dragging:
                event.accept()
                return True
        return False


class SlidingStack(QtWidgets.QWidget):
    # Move the actual pages inside a clipped container. This keeps camera feeds
    # and RViz live and avoids capturing or compositing a full-screen snapshot.
    transition_finished = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pages = []
        self._current = self._previous = None
        self._outgoing, self._starts = [], {}
        self._current_index = -1
        self._shift, self._progress = 0.0, 1.0
        self._transition = _animation(self, 300, self._advance)
        self._transition.setObjectName('pageTransition')
        self._transition.finished.connect(self._finish)

    def addWidget(self, widget):
        widget.setParent(self)
        widget.setMinimumWidth(0)
        widget.setMaximumWidth(16777215)
        self._pages.append(widget)
        widget.setGeometry(self.rect())
        if self._current is None:
            self._current, self._current_index = widget, 0
            self._starts[widget] = 0.0
            widget.show()
        else:
            widget.hide()
        return len(self._pages) - 1

    def currentIndex(self):
        return self._current_index

    def currentWidget(self):
        return self._current

    def setCurrentWidget(self, widget, direction=None, animated=True):
        self.setCurrentIndex(self._pages.index(widget), direction, animated)

    def setCurrentIndex(self, index, direction=None, animated=True):
        if not 0 <= index < len(self._pages):
            raise IndexError('Page index is out of range')
        incoming = self._pages[index]
        if incoming is self._current:
            return
        running = self._transition.state() == QtCore.QAbstractAnimation.Running
        if running:
            self._transition.stop()
        # Keep the positions of every page already on screen. A quick back tap
        # or a third destination then continues from the visible composition.
        visible = self._outgoing + ([self._current] if self._current else [])
        width = max(1, self.width())
        starts = {widget: widget.x() / width for widget in visible}
        if incoming not in starts:
            starts[incoming] = ((1 if index > self._current_index else -1) if direction is None else
                                (1 if direction >= 0 else -1))
        self._previous, self._current = self._current, incoming
        self._current_index = index
        self._outgoing = [widget for widget in visible if widget is not incoming]
        self._starts = {widget: starts[widget] for widget in self._outgoing + [incoming]}
        self._shift, self._progress = -starts[incoming], 0.0
        for widget in self._outgoing:
            widget.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        self._current.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, False)
        self._current.show()
        self._current.raise_()
        if not animated or not self.isVisible() or self.width() < 2:
            self._finish()
            return
        # Reconfiguring a stopped QVariantAnimation can emit valueChanged using
        # its old timeline. Block those intermediate values to avoid a jump.
        self._transition.blockSignals(True)
        self._transition.setDuration(max(80, round(300 * abs(self._shift))))
        self._transition.setStartValue(0.0)
        self._transition.setEndValue(1.0)
        self._transition.setCurrentTime(0)
        self._transition.blockSignals(False)
        self._place_pages()
        self._transition.start()

    def _advance(self, value):
        self._progress = float(value)
        self._place_pages()

    def _place_pages(self):
        width = self.width()
        for widget, start in self._starts.items():
            widget.setGeometry(round((start + self._shift * self._progress) * width), 0, width, self.height())

    def _finish(self):
        for widget in self._outgoing:
            widget.hide()
            widget.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, False)
        self._outgoing.clear()
        self._previous = None
        self._progress = 1.0
        if self._current:
            self._starts = {self._current: 0.0}
            self._shift = 0.0
            self._current.setGeometry(self.rect())
            self._current.show()
        self.transition_finished.emit(self._current_index)

    def finish_transition(self):
        if self._transition.state() == QtCore.QAbstractAnimation.Running:
            self._transition.stop()
            self._finish()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._place_pages()


class NavigationButton(QtWidgets.QPushButton):
    def __init__(self, text):
        super().__init__(text)
        self.setCheckable(True)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setMinimumSize(112, 54)
        self._selection, self._hover, self._press, self._keyboard_focus = 0.0, 0.0, 0.0, False
        self._icon_key, self._icon_normal, self._icon_selected = None, QtGui.QPixmap(), QtGui.QPixmap()
        self._selection_animation = _animation(self, 260, self._set_selection)
        self._hover_animation = _animation(self, 180, self._set_hover)
        self._press_animation = _animation(self, 120, self._set_press)
        self.toggled.connect(lambda checked: _animate(self._selection_animation, self._selection, 1.0 if checked else 0.0))
        self.pressed.connect(lambda: _animate(self._press_animation, self._press, 1.0))
        self.released.connect(lambda: _animate(self._press_animation, self._press, 0.0))

    def _set_selection(self, value):
        self._selection = value
        self.update()

    def _set_hover(self, value):
        self._hover = value
        self.update()

    def _set_press(self, value):
        self._press = value
        self.update()

    def focusInEvent(self, event):
        self._keyboard_focus = event.reason() in (QtCore.Qt.TabFocusReason, QtCore.Qt.BacktabFocusReason)
        super().focusInEvent(event)

    def focusOutEvent(self, event):
        self._keyboard_focus = False
        super().focusOutEvent(event)

    def event(self, event):
        if event.type() in (QtCore.QEvent.Enter, QtCore.QEvent.Leave):
            _animate(self._hover_animation, self._hover, 1.0 if event.type() == QtCore.QEvent.Enter else 0.0)
        return super().event(event)

    def _icons(self):
        # Recolor once per icon or theme change; painting an animation frame
        # then only blends the cached pixmaps.
        key = (self.icon().cacheKey(), white_theme())
        if key != self._icon_key:
            source = self.icon().pixmap(QtCore.QSize(20, 20))

            def tinted(shade):
                pixmap = QtGui.QPixmap(source.size())
                pixmap.setDevicePixelRatio(source.devicePixelRatioF())
                pixmap.fill(QtCore.Qt.transparent)
                painter = QtGui.QPainter(pixmap)
                painter.drawPixmap(0, 0, source)
                painter.setCompositionMode(QtGui.QPainter.CompositionMode_SourceIn)
                painter.fillRect(pixmap.rect(), QtGui.QColor(shade))
                painter.end()
                return pixmap

            self._icon_normal = tinted('#535b60' if key[1] else '#aab2b8')
            self._icon_selected = tinted('#f7f9f9' if key[1] else '#0f1419')
            self._icon_key = key
        return self._icon_normal, self._icon_selected

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        bounds = QtCore.QRectF(self.rect().adjusted(4, 6, -4, -6))
        overlay = 0 if white_theme() else 255
        painter.setPen(QtCore.Qt.NoPen)
        if self._hover > 0:
            painter.setBrush(QtGui.QColor(overlay, overlay, overlay, round(14 * self._hover)))
            painter.drawRoundedRect(bounds, 22, 22)
        if self._selection > 0:
            selected = color('#eff3f4', '#101417')
            selected.setAlpha(round(255 * self._selection))
            painter.setBrush(selected)
            painter.drawRoundedRect(bounds, 22, 22)
        painter.translate(0, self._press)
        font = painter.font()
        font.setPixelSize(14)
        font.setWeight(QtGui.QFont.DemiBold)
        painter.setFont(font)
        width = painter.fontMetrics().horizontalAdvance(self.text())
        left = (self.width() - width - 30) // 2
        icon_rect = QtCore.QRect(left, (self.height() - 20) // 2, 20, 20)
        normal_icon, selected_icon = self._icons()
        painter.setOpacity(1.0 - self._selection)
        painter.drawPixmap(icon_rect, normal_icon)
        painter.setOpacity(self._selection)
        painter.drawPixmap(icon_rect, selected_icon)
        painter.setOpacity(1.0)
        normal, selected = color('#aab2b8', '#535b60'), color('#0f1419', '#f7f9f9')
        text_color = QtGui.QColor.fromRgbF(*(normal.getRgbF()[i] +
                                            (selected.getRgbF()[i] - normal.getRgbF()[i]) * self._selection
                                            for i in range(3)))
        painter.setPen(text_color if self.isEnabled() else color('#71767b', '#858585'))
        painter.drawText(QtCore.QRect(left + 30, 0, width + 2, self.height()), QtCore.Qt.AlignVCenter, self.text())
        if self.hasFocus() and self._keyboard_focus:
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.setPen(QtGui.QPen(QtGui.QColor('#a5a5a5'), 1, QtCore.Qt.DashLine))
            painter.drawRoundedRect(bounds.adjusted(3, 3, -3, -3), 9, 9)


class LayerButton(QtWidgets.QToolButton):
    def __init__(self, name, accent):
        super().__init__()
        self.setText(name)
        self.setCheckable(True)
        self.setChecked(True)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setMinimumSize(56, 44)
        self._accent, self._rate, self._hover, self._keyboard_focus = QtGui.QColor(accent), '—', 0.0, False
        self._checked_progress = 1.0
        self._animation = _animation(self, 160, self._set_hover)
        self._check_animation = _animation(self, 200, self._set_checked_progress)
        self.toggled.connect(lambda checked: _animate(self._check_animation, self._checked_progress, 1.0 if checked else 0.0))

    def _set_hover(self, value):
        self._hover = value
        self.update()

    def _set_checked_progress(self, value):
        self._checked_progress = value
        self.update()

    def set_rate(self, value):
        next_value = (f'{value:.1f} Hz' if isinstance(value, (int, float)) and not isinstance(value, bool) and
                      math.isfinite(value) else '—')
        if self._rate != next_value:
            self._rate = next_value
            self.update()

    def focusInEvent(self, event):
        self._keyboard_focus = event.reason() in (QtCore.Qt.TabFocusReason, QtCore.Qt.BacktabFocusReason)
        super().focusInEvent(event)

    def focusOutEvent(self, event):
        self._keyboard_focus = False
        super().focusOutEvent(event)

    def event(self, event):
        if event.type() in (QtCore.QEvent.Enter, QtCore.QEvent.Leave):
            _animate(self._animation, self._hover, 1.0 if event.type() == QtCore.QEvent.Enter else 0.0)
        return super().event(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        overlay = 0 if white_theme() else 255
        if self._hover > 0:
            painter.fillRect(self.rect(), QtGui.QColor(overlay, overlay, overlay, round(12 * self._hover)))
        font = painter.font()
        font.setPixelSize(13)
        font.setWeight(QtGui.QFont.Medium)
        painter.setFont(font)
        painter.setPen(color('#e7e9ea', '#252525') if self.isChecked() else color('#62686d', '#898989'))
        painter.drawText(QtCore.QRect(2, 1, self.width() - 4, 22), QtCore.Qt.AlignCenter, self.text())
        font.setPixelSize(11)
        painter.setFont(font)
        painter.setPen(color('#93989b', '#64686b'))
        painter.drawText(QtCore.QRect(2, 22, self.width() - 4, 17), QtCore.Qt.AlignCenter, self._rate)
        if self._checked_progress > 0:
            indicator = 24 * self._checked_progress
            painter.fillRect(QtCore.QRectF((self.width() - indicator) / 2, self.height() - 4, indicator, 2),
                             self._accent.darker(190) if white_theme() else self._accent)
        if self.hasFocus() and self._keyboard_focus:
            painter.setPen(QtGui.QPen(QtGui.QColor('#b7b7b7'), 1, QtCore.Qt.DashLine))
            painter.drawRect(self.rect().adjusted(3, 3, -4, -4))


class ToolRow(QtWidgets.QPushButton):
    def __init__(self, title, icon):
        super().__init__(title)
        self.setIcon(icon)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setMinimumHeight(72)
        self._hover, self._keyboard_focus = 0.0, False
        self._animation = _animation(self, 180, self._set_hover)

    def _set_hover(self, value):
        self._hover = value
        self.update()

    def focusInEvent(self, event):
        self._keyboard_focus = event.reason() in (QtCore.Qt.TabFocusReason, QtCore.Qt.BacktabFocusReason)
        super().focusInEvent(event)

    def focusOutEvent(self, event):
        self._keyboard_focus = False
        super().focusOutEvent(event)

    def event(self, event):
        if event.type() in (QtCore.QEvent.Enter, QtCore.QEvent.Leave):
            _animate(self._animation, self._hover, 1.0 if event.type() == QtCore.QEvent.Enter else 0.0)
        return super().event(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        overlay = 0 if white_theme() else 255
        if self._hover > 0:
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(overlay, overlay, overlay, round(15 * self._hover)))
            painter.drawRoundedRect(self.rect().adjusted(0, 3, 0, -3), 12, 12)
        painter.setPen(color('#24282b', '#dedede'))
        painter.drawLine(50, self.height() - 1, self.width() - 12, self.height() - 1)
        self.icon().paint(painter, QtCore.QRect(14, (self.height() - 22) // 2, 22, 22))
        font = painter.font()
        font.setPixelSize(16)
        font.setWeight(QtGui.QFont.DemiBold)
        painter.setFont(font)
        painter.setPen(color('#f0f0f0', '#171717'))
        painter.drawText(QtCore.QRect(50, 0, self.width() - 90, self.height()), QtCore.Qt.AlignVCenter, self.text())
        painter.setPen(QtGui.QPen(color('#a0a0a0', '#606060'), 1.5, QtCore.Qt.SolidLine,
                                  QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
        offset = round(3 * self._hover)
        painter.drawLine(self.width() - 24 + offset, self.height() // 2 - 5, self.width() - 19 + offset, self.height() // 2)
        painter.drawLine(self.width() - 19 + offset, self.height() // 2, self.width() - 24 + offset, self.height() // 2 + 5)
        if self.hasFocus() and self._keyboard_focus:
            painter.setPen(QtGui.QPen(QtGui.QColor('#b7b7b7'), 1, QtCore.Qt.DashLine))
            painter.drawRect(self.rect().adjusted(3, 3, -4, -4))


class CameraButton(QtWidgets.QPushButton):
    def __init__(self, title, accent):
        super().__init__()
        self.title, self._accent, self.summary = title, QtGui.QColor(accent), ''
        self.setMinimumHeight(90)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self._highlight, self._hover = 0.0, 0.0
        self._previous_summary, self._summary_progress = '', 1.0
        self._status_animation = _animation(self, 320, self._set_highlight)
        self._summary_animation = _animation(self, 280, self._set_summary_progress)
        self._summary_animation.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self._hover_animation = _animation(self, 180, self._set_hover)
        self.set_summary('Waiting')

    def _set_highlight(self, value):
        self._highlight = value
        self.update()

    def _set_hover(self, value):
        self._hover = value
        self.update()

    def _set_summary_progress(self, value):
        self._summary_progress = value
        self.update()

    def set_summary(self, summary):
        if self.summary != summary:
            previous = self.summary
            self.summary = summary
            self.setAccessibleName(f'{self.title} camera: {summary}')
            _animate(self._status_animation, 1.0, 0.0)
            if self.isVisible() and self._summary_animation.state() != QtCore.QAbstractAnimation.Running:
                self._previous_summary = previous
                _animate(self._summary_animation, 0.0, 1.0)
            self.update()

    def event(self, event):
        if event.type() in (QtCore.QEvent.Enter, QtCore.QEvent.Leave):
            _animate(self._hover_animation, self._hover, 1.0 if event.type() == QtCore.QEvent.Enter else 0.0)
        return super().event(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        hover = self._hover if self.isEnabled() else 0.0
        white = white_theme()
        shade = round((224 if self.isDown() else 246 - 14 * hover) if white else
                      (38 if self.isDown() else 19 + 12 * hover))
        painter.setBrush(QtGui.QColor(shade, shade, shade))
        edge = color('#a4a4a4', '#666666') if self.hasFocus() else color('#303438', '#d6d9da')
        accent = self._accent.darker(160) if white else self._accent
        if self._highlight > 0:
            edge = QtGui.QColor.fromRgbF(*(edge.getRgbF()[i] + (accent.getRgbF()[i] - edge.getRgbF()[i]) *
                                          self._highlight * 0.65 for i in range(3)))
        painter.setPen(QtGui.QPen(edge, 1))
        painter.drawRoundedRect(self.rect().adjusted(1, 1, -1, -1), 10, 10)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(accent)
        painter.drawEllipse(QtCore.QPointF(17, 27), 3, 3)
        font = painter.font()
        font.setPixelSize(15)
        font.setWeight(QtGui.QFont.DemiBold)
        painter.setFont(font)
        painter.setPen(color('#f2f2f2', '#171717') if self.isEnabled() else color('#b1b1b1', '#777777'))
        painter.drawText(QtCore.QRect(26, 12, self.width() - 38, 30), QtCore.Qt.AlignVCenter, self.title)
        font.setPixelSize(12)
        font.setWeight(QtGui.QFont.Normal)
        painter.setFont(font)
        painter.setPen(color('#aaaaaa', '#585858'))
        rect = QtCore.QRect(15, 49, self.width() - 30, 24)
        painter.save()
        painter.setClipRect(rect)
        outgoing = min(1.0, self._summary_progress * 2.0)
        incoming = max(0.0, (self._summary_progress - 0.5) * 2.0)
        if self._summary_progress < 1.0:
            painter.setOpacity(1.0 - outgoing)
            painter.drawText(rect.translated(0, round(-6 * outgoing)), QtCore.Qt.AlignVCenter,
                             painter.fontMetrics().elidedText(self._previous_summary, QtCore.Qt.ElideRight, rect.width()))
        painter.setOpacity(incoming)
        painter.drawText(rect.translated(0, round(6 * (1.0 - incoming))), QtCore.Qt.AlignVCenter,
                         painter.fontMetrics().elidedText(self.summary, QtCore.Qt.ElideRight, rect.width()))
        painter.restore()


class PreviewScene(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCursor(QtCore.Qt.OpenHandCursor)
        self.setAttribute(QtCore.Qt.WA_AcceptTouchEvents)
        self.yaw, self.pitch, self.distance = 0.9, 0.6, 12.0
        self._last_mouse = QtCore.QPoint()
        self._touch = []
        self.painted = False
        self._scan_angle = None
        self._visible = [True] * 5
        colors = ['#d2e0f0', '#4fc1e6', '#78d3a7', '#f0ad69', '#b49ae8']
        self._points = []
        for i in range(4200):
            a, b = (i * 137 % 997) / 997.0, (i * 311 % 991) / 991.0
            if i % 4 == 0:
                point = (-5 + 10 * a, -5 + 10 * b, 0)
            elif i % 4 == 1:
                point = (-5 + 10 * a, 4.5, 3.2 * b)
            elif i % 4 == 2:
                point = (-4.5, -5 + 10 * a, 3.2 * b)
            else:
                point = (3.5 + 0.8 * a, -3.5 + b, 1.8 * (i * 97 % 983) / 983.0)
            layer = i % 5
            tint = QtGui.QColor(colors[layer])
            tint.setAlpha(120 + i % 120)
            self._points.append((QtGui.QVector3D(*point), QtGui.QPen(tint, 1.5), layer))

    def set_layer_visible(self, display, visible):
        names = ['LiDAR'] + [camera[3] for camera in CAMERAS]
        if display in names:
            self._visible[names.index(display)] = bool(visible)
            self.update()

    def set_scan_angle(self, angle):
        self._scan_angle = angle
        self.update()

    def rotate_by(self, delta):
        self.yaw += delta.x() * 0.008
        self.pitch = max(0.12, min(1.45, self.pitch + delta.y() * 0.006))
        self.update()

    def zoom_by(self, factor):
        self.distance = max(5.0, min(18.0, self.distance * factor))
        self.update()

    def event(self, event):
        if event.type() in (QtCore.QEvent.TouchBegin, QtCore.QEvent.TouchUpdate):
            points = sorted(((point.id(), point.pos()) for point in event.touchPoints()
                             if point.state() != QtCore.Qt.TouchPointReleased), key=lambda item: item[0])
            if [p[0] for p in points] == [p[0] for p in self._touch]:
                if len(points) == 1:
                    self.rotate_by(points[0][1] - self._touch[0][1])
                elif len(points) == 2:
                    before = QtCore.QLineF(self._touch[0][1], self._touch[1][1]).length()
                    after = QtCore.QLineF(points[0][1], points[1][1]).length()
                    if before >= 8 and after >= 8:
                        self.zoom_by(before / after)
            self._touch = points
            event.accept()
            return True
        if event.type() in (QtCore.QEvent.TouchEnd, QtCore.QEvent.TouchCancel):
            self._touch = []
            event.accept()
            return True
        return super().event(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#000000'))
        target = QtGui.QVector3D(0, 0, 0.5)
        eye = target + QtGui.QVector3D(self.distance * math.cos(self.pitch) * math.cos(self.yaw),
                                       self.distance * math.cos(self.pitch) * math.sin(self.yaw),
                                       self.distance * math.sin(self.pitch))
        forward = (target - eye).normalized()
        right = QtGui.QVector3D.crossProduct(forward, QtGui.QVector3D(0, 0, 1)).normalized()
        up = QtGui.QVector3D.crossProduct(right, forward).normalized()
        focal, center = min(self.width(), self.height()) * 0.88, QtCore.QPointF(self.width() * 0.5, self.height() * 0.53)

        def project(point):
            relative = point - eye
            depth = max(0.1, QtGui.QVector3D.dotProduct(relative, forward))
            return center + QtCore.QPointF(focal * QtGui.QVector3D.dotProduct(relative, right) / depth,
                                           -focal * QtGui.QVector3D.dotProduct(relative, up) / depth)
        painter.setPen(QtGui.QPen(QtGui.QColor(100, 100, 100, 85), 1))
        for index in range(-6, 7):
            painter.drawLine(project(QtGui.QVector3D(index, -6, 0)), project(QtGui.QVector3D(index, 6, 0)))
            painter.drawLine(project(QtGui.QVector3D(-6, index, 0)), project(QtGui.QVector3D(6, index, 0)))
        if self._scan_angle is not None:
            # Render a soft sector with fine range arcs, rather than a fan of
            # separate lines. This mirrors the native RViz activity indicator.
            def scan_point(radius, phase):
                return project(QtGui.QVector3D(radius * math.cos(phase), radius * math.sin(phase), 0.05))

            step = SCAN_WIDTH / SCAN_SEGMENTS
            painter.setPen(QtCore.Qt.NoPen)
            for index in range(SCAN_SEGMENTS):
                first = self._scan_angle - SCAN_WIDTH + index * step
                second = first + step
                strength = ((index + 0.5) / SCAN_SEGMENTS) ** 1.2
                painter.setBrush(QtGui.QColor(78, 171, 230, round(58 * strength)))
                painter.drawPolygon(QtGui.QPolygonF((scan_point(SCAN_INNER_RADIUS, first), scan_point(SCAN_RADIUS, first),
                                                     scan_point(SCAN_RADIUS, second), scan_point(SCAN_INNER_RADIUS, second))))
            painter.setBrush(QtCore.Qt.NoBrush)
            for radius in SCAN_RINGS:
                for index in range(SCAN_SEGMENTS):
                    first = self._scan_angle - SCAN_WIDTH + index * step
                    strength = ((index + 0.5) / SCAN_SEGMENTS) ** 1.2
                    painter.setPen(QtGui.QPen(QtGui.QColor(128, 205, 240, round(130 * strength)), 1))
                    painter.drawLine(scan_point(radius, first), scan_point(radius, first + step))
            painter.setPen(QtGui.QPen(QtGui.QColor(112, 180, 215, 35), 1))
            painter.drawPolyline(QtGui.QPolygonF([scan_point(SCAN_RADIUS, index * 2 * math.pi / SCAN_RIM_SEGMENTS)
                                                 for index in range(SCAN_RIM_SEGMENTS + 1)]))
            painter.setPen(QtGui.QPen(QtGui.QColor(154, 221, 244, 185), 1.3))
            painter.drawLine(scan_point(SCAN_INNER_RADIUS, self._scan_angle), scan_point(SCAN_RADIUS, self._scan_angle))
        for point, pen, layer in self._points:
            if self._visible[layer]:
                painter.setPen(pen)
                painter.drawPoint(project(point))
        car = [QtGui.QVector3D(*point) for point in ((-0.7, -0.45, 0.1), (1.2, -0.45, 0.1),
               (1.2, 0.45, 0.1), (-0.7, 0.45, 0.1), (-0.7, -0.45, 0.7), (1.2, -0.45, 0.7),
               (1.2, 0.45, 0.7), (-0.7, 0.45, 0.7))]
        painter.setPen(QtGui.QPen(QtGui.QColor('#b6d5df'), 2))
        for index in range(4):
            painter.drawLine(project(car[index]), project(car[(index + 1) % 4]))
            painter.drawLine(project(car[index + 4]), project(car[(index + 1) % 4 + 4]))
            painter.drawLine(project(car[index]), project(car[index + 4]))
        self.painted = True

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and not event.modifiers() & QtCore.Qt.ShiftModifier:
            self._last_mouse = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()

    def mouseMoveEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton and not event.modifiers() & QtCore.Qt.ShiftModifier:
            self.rotate_by(event.pos() - self._last_mouse)
            self._last_mouse = event.pos()
            event.accept()

    def mouseReleaseEvent(self, event):
        self.setCursor(QtCore.Qt.OpenHandCursor)

    def wheelEvent(self, event):
        if not event.modifiers() & QtCore.Qt.ShiftModifier:
            self.zoom_by(0.85 ** (event.angleDelta().y() / 120.0))


# Keep the theme rules with the controls that use them.
BLACK_STYLE = '''
      QWidget { font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; color: #eff3f4; font-size: 14px; }
      QWidget#root { background: #000000; }
      QFrame#header { background: #08090a; border: 0; border-bottom: 1px solid #202428; }
      QLabel#connection, QLabel#sceneStatus { color: #aaaaaa; background: transparent; padding: 3px 0; font-size: 12px; }
      QLabel#recordingHeader { color: #ff8585; background: transparent; font-size: 12px; font-weight: 600; }
      QLabel#clock { color: #ededed; font-size: 14px; font-weight: 600; }
      QLabel#dateHeader { color: #a8a8a8; font-size: 13px; }
      QLabel#weatherHeader { color: #dddddd; font-size: 13px; }
      QPushButton#internetButton { background: transparent; border: 0; padding: 0 4px; font-size: 12px; }
      QPushButton#internetButton:hover { background: #181b1e; border-radius: 12px; }
      QFrame#viewFrame { background: #000000; border: 0; }
      QPushButton#fullscreenButton { background: #15181b; border: 1px solid #2f3336; border-radius: 12px; padding: 0; }
      QPushButton#fullscreenButton:hover { background: #202428; border-color: #5a6268; }
      QPushButton#fullscreenButton:pressed { background: #303539; }
      QFrame#legend { background: #08090a; border-top: 1px solid #202428; }
      QLabel#sectionCaption, QLabel#hint { color: #a0a7ad; font-size: 12px; }
      QLabel#layerCaption { color: #8b8f92; font-size: 10px; font-weight: 600; letter-spacing: 0.6px; }
      QFrame#navigation { background: #08090a; border-top: 1px solid #202428; }
      QPushButton#navButton { border: 0; background: transparent; }
      QPushButton#windowButton { color: #e6e6e6; background: #161616; border: 1px solid #3a3a3a; border-radius: 5px; min-height: 40px; padding: 0 12px; font-weight: 500; }
      QPushButton#windowButton:hover { background: #262626; border-color: #666666; }
      QPushButton#windowButton:pressed { background: #303030; }
      QPushButton#themeBlack, QPushButton#themeWhite { color: #c8cdd0; background: #15181b; border: 1px solid #2f3336; border-radius: 20px; padding: 0 10px; }
      QPushButton#themeBlack:hover, QPushButton#themeWhite:hover { background: #24292c; }
      QPushButton#themeBlack:checked, QPushButton#themeWhite:checked { color: #0f1419; background: #eff3f4; border-color: #eff3f4; }
      QFrame#drawer, QWidget#cameraGallery, QFrame#vehicleStatusPanel, QWidget#recordingPanel,
      QFrame#toolsPanel, QWidget#updatePanel, QWidget#networkPanel { background: #08090a; }
      QDialog#cameraFocusDialog { background: #000000; }
      QLabel#galleryTitle { color: #f7f9f9; font-size: 21px; font-weight: 700; }
      QLabel#feedCount { color: #999999; background: transparent; font-size: 12px; }
      QScrollArea#drawerArea, QScrollArea#statusArea, QScrollArea#cameraArea, QScrollArea#recordingArea,
      QScrollArea#toolsArea, QScrollArea#updateArea, QScrollArea#networkArea { background: #08090a; border: 0; border-left: 1px solid #2f3336; }
      QComboBox#recordingCategory, QLineEdit#recordingSearch, QLineEdit#recordingName, QComboBox#networkCapturePort { background: #16181c; border: 1px solid #2f3336; border-radius: 10px; padding: 8px; }
      QComboBox#recordingCategory QAbstractItemView { background: #16181c; color: #eff3f4; selection-background-color: #343a3f; }
      QTreeWidget#recordingTopics { background: #0d0f11; border: 1px solid #2f3336; border-radius: 10px; outline: 0; }
      QTreeWidget#recordingTopics::item { padding: 4px 2px; }
      QTreeWidget#recordingTopics::item:selected { background: #3a3a3a; }
      QTreeWidget#recordingTopics::indicator { width: 20px; height: 20px; border: 1px solid #777777; border-radius: 3px; background: #171717; }
      QTreeWidget#recordingTopics::indicator:checked { background: #3975e8; border-color: #3975e8; }
      QTreeWidget#recordingTopics QHeaderView::section { background: #16181c; color: #a8afb4; border: 0; padding: 7px; }
      QFrame#recordingStatusCard { background: #16181c; border: 1px solid #2f3336; border-radius: 10px; }
      QLabel#recordingState { color: #eff3f4; font-size: 15px; font-weight: 600; }
      QLabel#recordingDirectory, QLabel#recordingMessage { color: #aab8c2; font-size: 12px; }
      QLabel#updateStatus { color: #f0f0f0; font-size: 20px; font-weight: 600; }
      QLabel#updateDetail { color: #a0a7ad; font-size: 13px; }
      QLabel#networkWeather, QLabel#networkPortDetail, QLabel#networkCaptureStatus { color: #a0a7ad; font-size: 12px; }
      QLabel#networkInternet { color: #dedede; font-size: 13px; }
      QTreeWidget#networkPorts { background: #0d0f11; border: 1px solid #2f3336; border-radius: 10px; outline: 0; }
      QTreeWidget#networkPorts::item:selected { background: #3a3a3a; }
      QTreeWidget#networkPorts QHeaderView::section { background: #16181c; color: #a8afb4; border: 0; padding: 7px; }
      QPlainTextEdit#networkCaptureOutput { background: #0d0f11; border: 1px solid #2f3336; border-radius: 10px; color: #d5d9dc; font-family: monospace; font-size: 11px; padding: 7px; }
      QPushButton#networkCaptureButton { background: #16181c; border: 1px solid #41484d; border-radius: 20px; color: #eff3f4; padding: 8px 20px; font-weight: 600; }
      QPushButton#networkCaptureButton:hover { background: #272c30; }
      QPushButton#networkCaptureButton:disabled { color: #777777; border-color: #383838; }
      QLabel#updateChanges { color: #dedede; background: #16181c; border: 1px solid #2f3336; border-radius: 10px; padding: 14px; font-size: 13px; }
      QComboBox#detailCamera { background: #16181c; border: 1px solid #2f3336; border-radius: 10px; padding: 9px; min-height: 28px; }
      QComboBox#detailCamera:hover { border-color: #777777; }
      QComboBox#detailCamera QAbstractItemView { background: #141414; color: #eeeeee; selection-background-color: #454545; selection-color: #ffffff; }
      QTreeWidget#tfTree { background: #0d0f11; border: 1px solid #2f3336; border-radius: 10px; color: #eeeeee; outline: 0; }
      QTreeWidget#tfTree::item { padding: 3px; }
      QTreeWidget#tfTree::item:selected { background: #3a3a3a; }
      QTreeWidget#tfTree QHeaderView::section { background: #141414; color: #eeeeee; border: 0; padding: 7px; }
      QLabel#calibrationQuality, QLabel#statusHint, QLabel#recordingTopicDetail { color: #a0a7ad; font-size: 12px; }
      QLabel#transformSource { color: #8ecdf6; font-size: 12px; font-weight: 600; }
      QLabel#cloudMatrix, QLabel#mountMatrix, QLabel#tfMatrix { color: #eeeeee; background: #0d0f11; border: 1px solid #2f3336; border-radius: 10px; padding: 10px; font-family: monospace; font-size: 11px; }
      QLabel#drawerTitle { color: #f5f5f5; font-size: 24px; font-weight: 600; }
      QLabel#sectionTitle { color: #eeeeee; font-size: 15px; font-weight: 600; }
      QLabel#bodyText { color: #a0a7ad; font-size: 14px; }
      QPushButton#closeButton { color: #e7e9ea; background: transparent; border: 0; border-radius: 4px; font-size: 24px; }
      QPushButton#closeButton:hover { color: #ffffff; background: #181818; }
      QPushButton#primaryButton, QPushButton#recordingStart, QPushButton#updateAction { background: #f7f9f9; border: 1px solid #f7f9f9; border-radius: 23px; color: #0f1419; padding: 11px; font-weight: 700; }
      QPushButton#primaryButton:hover, QPushButton#recordingStart:hover, QPushButton#updateAction:hover { background: #dce0e1; border-color: #dce0e1; }
      QPushButton#primaryButton:pressed, QPushButton#recordingStart:pressed, QPushButton#updateAction:pressed { background: #c4c9cb; border-color: #c4c9cb; }
      QPushButton#primaryButton:disabled, QPushButton#recordingStart:disabled, QPushButton#updateAction:disabled { background: #25292c; border-color: #303539; color: #8b9499; }
      QPushButton#secondaryButton, QPushButton#recordingStop { background: #16181c; border: 1px solid #41484d; border-radius: 20px; color: #eff3f4; padding: 10px; font-weight: 600; }
      QPushButton#secondaryButton:hover, QPushButton#recordingStop:hover { background: #272c30; border-color: #667078; }
      QPushButton#secondaryButton:pressed, QPushButton#recordingStop:pressed { background: #33393d; }
      QPushButton#secondaryButton:disabled, QPushButton#recordingStop:disabled { background: #171a1d; border-color: #2f3336; color: #7c858a; }
      QPushButton#recordingStop:enabled { color: #ff9a9e; background: #290d10; border-color: #9f3c43; }
      QPushButton#recordingStop:enabled:hover { color: #ffffff; background: #542025; border-color: #d15a61; }
      QPushButton#recordingStop:enabled:pressed { background: #6a262c; }
      QPushButton#textButton { border: 0; color: #9abcf8; min-height: 40px; }
      QPushButton#textButton:hover { color: #cfddfa; }
      QFrame#reviewFrame { background: #16181c; border: 1px solid #2f3336; border-radius: 12px; }
      QLabel#reviewTitle { color: #eff3f4; font-size: 15px; font-weight: 700; }
      QLabel#message { color: #f4bd73; font-size: 12px; }
      QFrame#metricCard { background: #16181c; border: 1px solid #2f3336; border-radius: 10px; }
      QLabel#metricCaption { color: #a0a7ad; font-size: 12px; }
      QLabel#vehicleMode, QLabel#vehicleSpeed, QLabel#vehicleBattery, QLabel#vehiclePosition,
      QLabel#commandThrottle, QLabel#commandSteering, QLabel#commandBrake { color: #f7f9f9; font-size: 17px; font-weight: 600; }
      QLabel#signalName { color: #e7e9ea; font-size: 12px; }
      QLabel#diagnosticStatus { color: #b5bdc3; font-size: 12px; }
      QScrollBar:vertical { background: #08090a; width: 14px; margin: 0; }
      QScrollBar::handle:vertical { background: #535b60; min-height: 30px; border-radius: 7px; }
      QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
      QScrollBar:horizontal { background: #08090a; height: 14px; margin: 0; }
      QScrollBar::handle:horizontal { background: #535b60; min-width: 30px; border-radius: 7px; }
      QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
      QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }
      QAbstractScrollArea::corner { background: #000000; }
      QToolTip { color: #e7e9ea; background: #121212; border: 1px solid #555555; padding: 5px; }
    '''

WHITE_STYLE = '''
      QWidget { color: #202124; }
      QWidget#root, QFrame#header, QFrame#legend, QFrame#navigation,
      QFrame#drawer, QWidget#cameraGallery, QFrame#vehicleStatusPanel, QWidget#recordingPanel,
      QFrame#toolsPanel, QWidget#updatePanel, QWidget#networkPanel, QDialog#cameraFocusDialog { background: #ffffff; }
      QFrame#header { border-bottom: 1px solid #dedede; }
      QFrame#legend, QFrame#navigation { border-top: 1px solid #dedede; }
      QLabel#connection, QLabel#sceneStatus, QLabel#dateHeader { color: #555b5e; }
      QLabel#recordingHeader { color: #b22230; }
      QLabel#clock, QLabel#weatherHeader { color: #202124; }
      QPushButton#internetButton:hover { background: #eeeeee; }
      QPushButton#fullscreenButton, QPushButton#windowButton {
        color: #202124; background: #f3f3f3; border-color: #c7c7c7;
      }
      QPushButton#fullscreenButton:hover, QPushButton#windowButton:hover {
        background: #e7e7e7; border-color: #929292;
      }
      QPushButton#fullscreenButton:pressed, QPushButton#windowButton:pressed { background: #dcdcdc; }
      QPushButton#themeBlack, QPushButton#themeWhite { color: #333333; background: #f3f3f3; border-color: #c7c7c7; }
      QPushButton#themeBlack:hover, QPushButton#themeWhite:hover { background: #e7e7e7; }
      QPushButton#themeBlack:checked, QPushButton#themeWhite:checked { color: #ffffff; background: #222222; border-color: #222222; }
      QLabel#sectionCaption, QLabel#hint, QLabel#layerCaption { color: #60666a; }
      QLabel#galleryTitle, QLabel#drawerTitle, QLabel#sectionTitle { color: #181a1b; }
      QLabel#feedCount, QLabel#bodyText { color: #5b6164; }
      QScrollArea#drawerArea, QScrollArea#statusArea, QScrollArea#cameraArea, QScrollArea#recordingArea,
      QScrollArea#toolsArea, QScrollArea#updateArea, QScrollArea#networkArea { background: #ffffff; border-left: 1px solid #dedede; }
      QComboBox#recordingCategory, QLineEdit#recordingSearch, QLineEdit#recordingName, QComboBox#networkCapturePort, QComboBox#detailCamera {
        color: #202124; background: #f5f5f5; border-color: #bcbcbc;
      }
      QComboBox#recordingCategory QAbstractItemView, QComboBox#detailCamera QAbstractItemView {
        background: #ffffff; color: #202124; selection-background-color: #dce8f8; selection-color: #202124;
      }
      QTreeWidget#recordingTopics, QTreeWidget#networkPorts, QTreeWidget#tfTree {
        color: #202124; background: #fafafa; border-color: #cccccc;
      }
      QTreeWidget#recordingTopics::item:selected, QTreeWidget#networkPorts::item:selected,
      QTreeWidget#tfTree::item:selected { color: #202124; background: #dce8f8; }
      QTreeWidget#recordingTopics::indicator { background: #ffffff; border-color: #858585; }
      QTreeWidget#recordingTopics::indicator:checked { background: #3975e8; border-color: #3975e8; }
      QTreeWidget#recordingTopics QHeaderView::section, QTreeWidget#networkPorts QHeaderView::section,
      QTreeWidget#tfTree QHeaderView::section { color: #50575a; background: #eeeeee; }
      QFrame#recordingStatusCard { background: #f5f5f5; border-color: #d0d0d0; }
      QLabel#recordingState, QLabel#updateStatus, QLabel#networkInternet { color: #202124; }
      QLabel#recordingDirectory, QLabel#recordingMessage, QLabel#updateDetail, QLabel#networkWeather,
      QLabel#networkPortDetail, QLabel#networkCaptureStatus, QLabel#calibrationQuality,
      QLabel#statusHint, QLabel#recordingTopicDetail { color: #555c60; }
      QPlainTextEdit#networkCaptureOutput, QLabel#updateChanges, QLabel#cloudMatrix,
      QLabel#mountMatrix, QLabel#tfMatrix { color: #202124; background: #f7f7f7; border-color: #cccccc; }
      QPushButton#primaryButton:enabled, QPushButton#recordingStart:enabled, QPushButton#updateAction:enabled {
        color: #ffffff; background: #101417; border-color: #101417;
      }
      QPushButton#primaryButton:enabled:hover, QPushButton#recordingStart:enabled:hover, QPushButton#updateAction:enabled:hover {
        background: #30373b; border-color: #30373b;
      }
      QPushButton#primaryButton:enabled:pressed, QPushButton#recordingStart:enabled:pressed, QPushButton#updateAction:enabled:pressed {
        background: #454e53; border-color: #454e53;
      }
      QPushButton#networkCaptureButton, QPushButton#secondaryButton, QPushButton#recordingStop {
        color: #202124; background: #f4f4f4; border-color: #aaaaaa;
      }
      QPushButton#networkCaptureButton:hover, QPushButton#secondaryButton:hover, QPushButton#recordingStop:hover {
        background: #e5e5e5; border-color: #858585;
      }
      QPushButton#networkCaptureButton:disabled, QPushButton#secondaryButton:disabled, QPushButton#recordingStop:disabled,
      QPushButton#primaryButton:disabled, QPushButton#recordingStart:disabled, QPushButton#updateAction:disabled {
        color: #737373; background: #e8e8e8; border-color: #cccccc;
      }
      QPushButton#recordingStop:enabled { color: #a5232d; background: #fff1f2; border-color: #d08b90; }
      QPushButton#recordingStop:enabled:hover { color: #841921; background: #ffe1e3; border-color: #b65a61; }
      QPushButton#recordingStop:enabled:pressed { background: #f5ced1; }
      QLabel#transformSource { color: #1c6399; }
      QPushButton#closeButton { color: #333333; }
      QPushButton#closeButton:hover { color: #111111; background: #eeeeee; }
      QPushButton#textButton { color: #245aaf; }
      QPushButton#textButton:hover { color: #173b78; }
      QFrame#reviewFrame, QFrame#metricCard { background: #f5f5f5; border-color: #d0d0d0; }
      QLabel#reviewTitle, QLabel#vehicleMode, QLabel#vehicleSpeed,
      QLabel#vehicleBattery, QLabel#vehiclePosition, QLabel#commandThrottle, QLabel#commandSteering,
      QLabel#commandBrake, QLabel#signalName { color: #202124; }
      QLabel#message { color: #895314; }
      QLabel#metricCaption, QLabel#diagnosticStatus { color: #555c60; }
      QScrollBar:vertical { background: #ffffff; }
      QScrollBar::handle:vertical { background: #a8a8a8; }
      QScrollBar:horizontal { background: #ffffff; }
      QScrollBar::handle:horizontal { background: #a8a8a8; }
      QAbstractScrollArea::corner { background: #ffffff; }
      QToolTip { color: #202124; background: #ffffff; border-color: #aaaaaa; }
    '''
