# Touch-friendly drawing board with editable sketches, history and PNG export.
import json
import math
import os
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets, sip


PAPER_WIDTH, PAPER_HEIGHT = 1600, 900
MAX_FILE_BYTES, MAX_ITEMS, MAX_POINTS = 20 * 1024 * 1024, 10000, 500000


def _white_theme():
    app = QtWidgets.QApplication.instance()
    return bool(app and app.property('vehicleHubWhiteTheme'))


def _folder():
    if Path('/vehicle_1811').is_dir():
        return '/vehicle_1811/sketches'
    return QtCore.QStandardPaths.writableLocation(QtCore.QStandardPaths.DocumentsLocation) or str(Path.home())


def _color_icon(color):
    pixmap = QtGui.QPixmap(24, 24)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing)
    painter.setPen(QtGui.QPen(QtGui.QColor('#a6a6a6'), 1))
    painter.setBrush(color)
    painter.drawEllipse(QtCore.QRectF(3, 3, 18, 18))
    painter.end()
    return QtGui.QIcon(pixmap)


class _Stroke(QtWidgets.QGraphicsPathItem):
    # A wider hit area makes thin lines practical to select with a finger.
    def shape(self):
        outline = QtGui.QPainterPathStroker()
        outline.setWidth(max(30.0, self.pen().widthF()))
        outline.setCapStyle(QtCore.Qt.RoundCap)
        outline.setJoinStyle(QtCore.Qt.RoundJoin)
        result = outline.createStroke(self.path())
        result.setFillRule(QtCore.Qt.WindingFill)
        return result

    def boundingRect(self):
        padding = max(0.0, (30.0 - self.pen().widthF()) / 2.0)
        return super().boundingRect().adjusted(-padding, -padding, padding, padding)


class _Text(QtWidgets.QGraphicsTextItem):
    def __init__(self, focus_changed, edited):
        super().__init__()
        self._focus_changed, self._edited = focus_changed, edited
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsFocusable, True)

    def focusInEvent(self, event):
        super().focusInEvent(event)
        self._focus_changed(self, True)

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        self._focus_changed(self, False)
        self._edited()


class Canvas(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('sketchCanvas')
        self._scene = QtWidgets.QGraphicsScene(self)
        self._scene.setSceneRect(0, 0, PAPER_WIDTH, PAPER_HEIGHT)
        self._scene.setItemIndexMethod(QtWidgets.QGraphicsScene.NoIndex)
        self.setScene(self._scene)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setViewportUpdateMode(QtWidgets.QGraphicsView.MinimalViewportUpdate)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setCursor(QtCore.Qt.CrossCursor)
        self.tool, self.color, self.pen_size = 'pen', QtGui.QColor('#171717'), 5
        self.changed = self.text_editing = None
        self._dragging = self._restoring = False
        self._last_point = QtCore.QPointF()
        self._path, self._stroke, self._editing = QtGui.QPainterPath(), None, None
        self._history, self._history_index = [[]], 0
        self._scene.selectionChanged.connect(self._notify)

    def shutdown(self):
        self._restoring = True
        self.changed = self.text_editing = None
        self._editing = None
        self._scene.clearFocus()
        self._scene.clear()

    def set_tool(self, tool):
        self.finish_editing()
        self.tool = tool
        self._scene.clearSelection()
        self._configure_items()
        self.setDragMode(QtWidgets.QGraphicsView.RubberBandDrag if tool == 'select' else QtWidgets.QGraphicsView.NoDrag)
        self.setCursor(QtCore.Qt.ArrowCursor if tool == 'select' else QtCore.Qt.IBeamCursor if tool == 'text' else QtCore.Qt.CrossCursor)
        self._notify()

    def set_color(self, color):
        if not color.isValid():
            return
        self.finish_editing()
        self.color = color
        for item in self._scene.selectedItems():
            if isinstance(item, QtWidgets.QGraphicsPathItem):
                pen = item.pen()
                pen.setColor(color)
                item.setPen(pen)
            elif isinstance(item, _Text):
                item.setDefaultTextColor(color)
        self._commit()

    def set_size(self, size, apply_selection):
        self.pen_size = max(2, min(30, size))
        if not apply_selection:
            return
        self.finish_editing()
        for item in self._scene.selectedItems():
            if isinstance(item, QtWidgets.QGraphicsPathItem):
                pen = item.pen()
                pen.setWidthF(self.pen_size)
                item.setPen(pen)
        self._commit()

    def finish_editing(self):
        if self._dragging:
            self._dragging, self._stroke = False, None
            self._commit()
        item = self._scene.focusItem()
        if isinstance(item, _Text):
            self._scene.setFocusItem(None)
            item.setTextInteractionFlags(QtCore.Qt.NoTextInteraction)

    def type_text(self, value):
        item = self._scene.focusItem()
        if isinstance(item, _Text):
            cursor = item.textCursor()
            cursor.insertText(value)
            item.setTextCursor(cursor)

    def backspace(self):
        item = self._scene.focusItem()
        if isinstance(item, _Text):
            cursor = item.textCursor()
            cursor.deletePreviousChar()
            item.setTextCursor(cursor)

    def undo(self):
        self.finish_editing()
        if self._history_index:
            self._history_index -= 1
            self._restore(self._history[self._history_index])
            self._notify()

    def redo(self):
        self.finish_editing()
        if self._history_index + 1 < len(self._history):
            self._history_index += 1
            self._restore(self._history[self._history_index])
            self._notify()

    def remove_selected(self):
        self.finish_editing()
        for item in self._scene.selectedItems():
            self._scene.removeItem(item)
            sip.delete(item)
        self._commit()

    def clear(self):
        self.finish_editing()
        self._scene.clear()
        self._commit()

    def _notify(self):
        if self.changed:
            self.changed()

    def _snapshot(self):
        document = []
        for item in self._scene.items(QtCore.Qt.AscendingOrder):
            position = item.pos()
            if isinstance(item, QtWidgets.QGraphicsPathItem):
                path = item.path()
                points = [[path.elementAt(i).x, path.elementAt(i).y] for i in range(path.elementCount())]
                document.append({'type': 'stroke', 'color': item.pen().color().name(QtGui.QColor.HexArgb),
                                 'x': position.x(), 'y': position.y(), 'width': item.pen().widthF(), 'points': points})
            elif isinstance(item, _Text):
                document.append({'type': 'text', 'color': item.defaultTextColor().name(QtGui.QColor.HexArgb),
                                 'x': position.x(), 'y': position.y(), 'text': item.toPlainText(),
                                 'font_size': item.font().pixelSize()})
        return document

    def _restore(self, document):
        # Only rebuild items for undo, redo or opening a file, never on pointer motion.
        self._restoring = True
        self._scene.clear()
        for entry in document:
            color = QtGui.QColor(entry['color'])
            if entry['type'] == 'text':
                item = self._make_text()
                font = item.font()
                font.setPixelSize(entry['font_size'])
                item.setFont(font)
                item.setPlainText(entry['text'])
                item.setDefaultTextColor(color)
            else:
                path = QtGui.QPainterPath()
                for index, (x, y) in enumerate(entry['points']):
                    path.moveTo(x, y) if index == 0 else path.lineTo(x, y)
                item = _Stroke()
                item.setPath(path)
                item.setPen(QtGui.QPen(color, entry['width'], QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
            item.setPos(entry['x'], entry['y'])
            self._scene.addItem(item)
        self._configure_items()
        self._restoring = False

    def _commit(self):
        if self._restoring or self._dragging:
            return
        document = self._snapshot()
        if self._history[self._history_index] == document:
            self._notify()
            return
        self._history = self._history[:self._history_index + 1]
        self._history.append(document)
        if len(self._history) > 40:
            self._history.pop(0)
        else:
            self._history_index += 1
        self._notify()

    @staticmethod
    def _number(value):
        return (isinstance(value, (int, float)) and not isinstance(value, bool) and
                abs(value) <= 100000 and math.isfinite(value))

    @staticmethod
    def _write_atomic(path, data):
        file = QtCore.QSaveFile(os.fspath(path))
        if not file.open(QtCore.QIODevice.WriteOnly):
            raise OSError(file.errorString())
        if file.write(data) != len(data) or not file.commit():
            raise OSError(file.errorString())

    def save(self, path):
        self.finish_editing()
        items = self._snapshot()
        if any(len(item['text']) > 10000 for item in items if item['type'] == 'text'):
            raise ValueError('A text box is too long to save.')
        if sum(len(item['points']) for item in items if item['type'] == 'stroke') > MAX_POINTS:
            raise ValueError('The sketch has too many points to save.')
        data = json.dumps({'version': 1, 'items': items}, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
        if len(items) > MAX_ITEMS or len(data) > MAX_FILE_BYTES:
            raise ValueError('The sketch is too large to save.')
        self._write_atomic(path, data)

    def load(self, path):
        with open(path, 'rb') as file:
            data = file.read(MAX_FILE_BYTES + 1)
        if len(data) > MAX_FILE_BYTES:
            raise ValueError('The sketch file is too large.')
        try:
            root = json.loads(data)
        except (ValueError, UnicodeDecodeError) as error:
            raise ValueError('This is not a supported sketch file.') from error
        if (not isinstance(root, dict) or isinstance(root.get('version'), bool) or root.get('version') != 1 or
                not isinstance(root.get('items'), list)):
            raise ValueError('This is not a supported sketch file.')
        items = root['items']
        if len(items) > MAX_ITEMS:
            raise ValueError('The sketch has too many objects.')
        total = 0
        for item in items:
            if (not isinstance(item, dict) or item.get('type') not in ('text', 'stroke') or
                    not isinstance(item.get('color'), str) or not QtGui.QColor(item['color']).isValid() or
                    not self._number(item.get('x')) or not self._number(item.get('y'))):
                raise ValueError('The sketch contains an invalid object.')
            if item['type'] == 'text':
                font_size = item.get('font_size')
                if (not isinstance(item.get('text'), str) or len(item['text']) > 10000 or
                        not self._number(font_size) or not 4 <= font_size <= 128):
                    raise ValueError('The sketch contains invalid text.')
                item['font_size'] = int(font_size + 0.5)
            else:
                points, width = item.get('points'), item.get('width')
                if not self._number(width) or not 1 <= width <= 64 or not isinstance(points, list):
                    raise ValueError('The sketch contains an invalid stroke.')
                total += len(points)
                if len(points) < 2 or total > MAX_POINTS:
                    raise ValueError('The sketch contains too many or too few points.')
                if any(not isinstance(point, list) or len(point) != 2 or not all(self._number(v) for v in point)
                       for point in points):
                    raise ValueError('The sketch contains an invalid point.')
        # Validate the entire file before replacing the board; undo restores it.
        self.finish_editing()
        self._restore(items)
        self._commit()

    def export_png(self, path):
        self.finish_editing()
        image = QtGui.QImage(PAPER_WIDTH, PAPER_HEIGHT, QtGui.QImage.Format_ARGB32_Premultiplied)
        image.fill(QtCore.Qt.white)
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        selected = self._scene.selectedItems()
        self._scene.clearSelection()
        self._scene.render(painter, QtCore.QRectF(0, 0, PAPER_WIDTH, PAPER_HEIGHT), self._scene.sceneRect())
        for item in selected:
            item.setSelected(True)
        painter.end()
        file = QtCore.QSaveFile(os.fspath(path))
        if not file.open(QtCore.QIODevice.WriteOnly):
            raise OSError(file.errorString())
        if not image.save(file, 'PNG') or not file.commit():
            raise OSError(file.errorString())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_view()

    def drawBackground(self, painter, rect):
        painter.fillRect(rect, QtGui.QColor('#eeeeee' if _white_theme() else '#101010'))
        painter.fillRect(self._scene.sceneRect(), QtCore.Qt.white)
        painter.setPen(QtGui.QPen(QtGui.QColor('#d2d2d2'), 1))
        painter.drawRect(self._scene.sceneRect())

    def mousePressEvent(self, event):
        if event.button() != QtCore.Qt.LeftButton:
            event.ignore()
            return
        point = self.mapToScene(event.pos())
        if not self._scene.sceneRect().contains(point):
            event.ignore()
            return
        if self.tool == 'select':
            super().mousePressEvent(event)
            return
        if self.tool == 'text':
            existing = self.itemAt(event.pos())
            if isinstance(existing, _Text):
                if self._editing is existing:
                    super().mousePressEvent(event)
                else:
                    self.finish_editing()
                    existing.setTextInteractionFlags(QtCore.Qt.TextEditorInteraction)
                    cursor = existing.textCursor()
                    cursor.movePosition(QtGui.QTextCursor.End)
                    existing.setTextCursor(cursor)
                    self._scene.setFocusItem(existing, QtCore.Qt.MouseFocusReason)
                    self.setFocus(QtCore.Qt.MouseFocusReason)
            else:
                self.finish_editing()
                item = self._make_text()
                item.setPlainText('Text')
                item.setDefaultTextColor(self.color)
                font = item.font()
                font.setPixelSize(38)
                item.setFont(font)
                item.setPos(point)
                self._scene.addItem(item)
                item.setTextInteractionFlags(QtCore.Qt.TextEditorInteraction)
                self._scene.setFocusItem(item, QtCore.Qt.MouseFocusReason)
                self.setFocus(QtCore.Qt.MouseFocusReason)
                cursor = item.textCursor()
                cursor.select(QtGui.QTextCursor.Document)
                item.setTextCursor(cursor)
            event.accept()
            return
        self.finish_editing()
        self._last_point, self._dragging = point, True
        if self.tool == 'pen':
            self._begin_segment(point)
        else:
            self._erase_segment(point, point)
        event.accept()

    def mouseMoveEvent(self, event):
        if self.tool == 'select':
            super().mouseMoveEvent(event)
            return
        if not self._dragging or not event.buttons() & QtCore.Qt.LeftButton:
            event.ignore()
            return
        raw = self.mapToScene(event.pos())
        point = QtCore.QPointF(max(0.0, min(raw.x(), PAPER_WIDTH)), max(0.0, min(raw.y(), PAPER_HEIGHT)))
        if QtCore.QLineF(self._last_point, point).length() < 1.0:
            return
        if self.tool == 'pen':
            if self._path.elementCount() >= 256:
                self._begin_segment(self._last_point)
            self._path.lineTo(point)
            self._stroke.setPath(self._path)
        elif self.tool == 'erase':
            self._erase_segment(self._last_point, point)
        self._last_point = point
        event.accept()

    def mouseReleaseEvent(self, event):
        if self.tool == 'select':
            super().mouseReleaseEvent(event)
            self._commit()
        elif self._dragging and event.button() == QtCore.Qt.LeftButton:
            self._dragging, self._stroke = False, None
            self._commit()
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    def _focus_changed(self, item, active):
        self._editing = item if active else None
        if self.text_editing:
            self.text_editing(active)
        self._update_view()

    def _make_text(self):
        return _Text(self._focus_changed, self._commit)

    def _begin_segment(self, point):
        self._path = QtGui.QPainterPath(point)
        self._path.lineTo(point + QtCore.QPointF(0.01, 0.01))
        self._stroke = _Stroke()
        self._stroke.setPath(self._path)
        self._stroke.setPen(QtGui.QPen(self.color, self.pen_size, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
        self._scene.addItem(self._stroke)

    def _erase_segment(self, start, end):
        path = QtGui.QPainterPath(start)
        path.lineTo(end if end != start else end + QtCore.QPointF(0.01, 0.01))
        eraser = QtGui.QPainterPathStroker()
        eraser.setWidth(max(12, self.pen_size * 3))
        eraser.setCapStyle(QtCore.Qt.RoundCap)
        for item in self._scene.items(eraser.createStroke(path), QtCore.Qt.IntersectsItemShape):
            if isinstance(item, (QtWidgets.QGraphicsPathItem, _Text)):
                self._scene.removeItem(item)
                sip.delete(item)

    def _configure_items(self):
        selectable = self.tool == 'select'
        for item in self._scene.items():
            item.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, selectable)
            item.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, selectable)
            if isinstance(item, _Text):
                item.setTextInteractionFlags(QtCore.Qt.NoTextInteraction)

    def _update_view(self):
        if not self._editing or sip.isdeleted(self._editing):
            self.fitInView(self._scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
            return
        fit = min(self.viewport().width() / PAPER_WIDTH, self.viewport().height() / PAPER_HEIGHT)
        zoom = max(0.7, fit)
        self.resetTransform()
        self.scale(zoom, zoom)
        self.centerOn(self._editing.sceneBoundingRect().center())


class SketchPad(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('sketchPage')
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        toolbar = QtWidgets.QFrame()
        toolbar.setObjectName('sketchToolbar')
        toolbar_layout = QtWidgets.QVBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(18, 8, 18, 8)
        toolbar_layout.setSpacing(6)
        first = QtWidgets.QHBoxLayout()
        first.setSpacing(5)
        title = QtWidgets.QLabel('Sketch')
        title.setObjectName('sketchTitle')
        first.addWidget(title)
        first.addSpacing(10)
        self.canvas = Canvas()
        self._tools = QtWidgets.QButtonGroup(self)
        for text, name, tool, tip in (('Pen', 'sketchPen', 'pen', 'Draw a stroke'),
                                      ('Erase', 'sketchErase', 'erase', 'Erase strokes or text'),
                                      ('Select', 'sketchSelect', 'select', 'Select and move objects'),
                                      ('Text', 'sketchText', 'text', 'Add or edit text')):
            button = self._button(text, name, 68)
            button.setCheckable(True)
            button.setToolTip(tip)
            self._tools.addButton(button)
            first.addWidget(button)
            button.clicked.connect(lambda checked=False, tool=tool: self.canvas.set_tool(tool))
            if tool == 'pen':
                button.setChecked(True)
        first.addStretch()
        self._undo = self._button('Undo', 'sketchUndo', 58)
        self._redo = self._button('Redo', 'sketchRedo', 58)
        self._remove = self._button('Delete', 'sketchDelete', 62)
        self._clear = self._button('Clear', 'sketchClear', 58)
        for button in (self._undo, self._redo, self._remove, self._clear):
            first.addWidget(button)
        self._undo.clicked.connect(self.canvas.undo)
        self._redo.clicked.connect(self.canvas.redo)
        self._remove.clicked.connect(self.canvas.remove_selected)
        self._clear.clicked.connect(self.canvas.clear)
        toolbar_layout.addLayout(first)
        second = QtWidgets.QHBoxLayout()
        second.setSpacing(5)
        self._colors = QtWidgets.QButtonGroup(self)
        self._custom = self._button('Color', 'sketchColorPicker', 70)
        self._custom.setIcon(_color_icon(QtGui.QColor('#171717')))
        self._custom.setIconSize(QtCore.QSize(16, 16))
        self._custom.setToolTip('Choose a custom color')
        for name, value in (('Black', '#171717'), ('Red', '#c8323e'), ('Blue', '#2864bf'),
                            ('Green', '#238453'), ('Orange', '#d47a27')):
            color = QtGui.QColor(value)
            button = QtWidgets.QToolButton()
            button.setObjectName('sketchColor' + name)
            button.setAccessibleName(name + ' pen')
            button.setToolTip(name + ' pen')
            button.setIcon(_color_icon(color))
            button.setIconSize(QtCore.QSize(24, 24))
            button.setFixedSize(38, 42)
            button.setCheckable(True)
            button.setCursor(QtCore.Qt.PointingHandCursor)
            self._colors.addButton(button)
            second.addWidget(button)
            if name == 'Black':
                button.setChecked(True)
            button.clicked.connect(lambda checked=False, color=color: self._set_color(color))
        second.addWidget(self._custom)
        self._custom.clicked.connect(self._choose_color)
        second.addSpacing(8)
        size_title = QtWidgets.QLabel('Size')
        size_title.setObjectName('sketchHint')
        second.addWidget(size_title)
        self._size = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._size.setObjectName('sketchSize')
        self._size.setRange(2, 30)
        self._size.setValue(5)
        self._size.setMinimumWidth(70)
        self._size.setMaximumWidth(180)
        self._size.setAccessibleName('Pen size')
        second.addWidget(self._size, 1)
        self._size_label = QtWidgets.QLabel('5')
        self._size_label.setObjectName('sketchHint')
        self._size_label.setFixedWidth(22)
        second.addWidget(self._size_label)
        self._size.valueChanged.connect(self._change_size)
        self._size.sliderReleased.connect(lambda: self.canvas.set_size(self._size.value(), True))
        second.addStretch()
        for text, name, width, action in (('Open', 'sketchOpen', 56, self._open),
                                          ('Save', 'sketchSave', 56, self._save),
                                          ('Export', 'sketchExport', 62, self._export)):
            button = self._button(text, name, width)
            second.addWidget(button)
            button.clicked.connect(action)
        toolbar_layout.addLayout(second)
        layout.addWidget(toolbar)
        layout.addWidget(self.canvas, 1)
        self._keyboard = self._make_keyboard()
        self._keyboard.hide()
        layout.addWidget(self._keyboard)
        self.canvas.text_editing = self._keyboard.setVisible
        self.canvas.changed = self._update_buttons
        self._update_buttons()
        self._undo_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence.Undo, self)
        self._redo_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence.Redo, self)
        self._undo_shortcut.activated.connect(self.canvas.undo)
        self._redo_shortcut.activated.connect(self.canvas.redo)
        self.apply_theme()

    @staticmethod
    def _button(text, name, width):
        button = QtWidgets.QPushButton(text)
        button.setObjectName(name)
        button.setMinimumSize(width, 42)
        button.setCursor(QtCore.Qt.PointingHandCursor)
        return button

    def _set_color(self, color):
        self.canvas.set_color(color)
        self._custom.setIcon(_color_icon(color))

    def _choose_color(self):
        color = QtWidgets.QColorDialog.getColor(self.canvas.color, self, 'Pen color')
        if color.isValid():
            self._colors.setExclusive(False)
            for button in self._colors.buttons():
                button.setChecked(False)
            self._colors.setExclusive(True)
            self._set_color(color)

    def _change_size(self, value):
        self._size_label.setText(str(value))
        self.canvas.set_size(value, not self._size.isSliderDown())

    def _make_keyboard(self):
        keyboard = QtWidgets.QFrame()
        keyboard.setObjectName('sketchKeyboard')
        layout = QtWidgets.QVBoxLayout(keyboard)
        layout.setContentsMargins(18, 6, 18, 6)
        layout.setSpacing(4)
        keys, letters, symbols = [], [], []
        for letter_row, symbol_row in zip(('qwertyuiop', 'asdfghjkl', 'zxcvbnm'),
                                           ('1234567890', '@#$%&*+-=', '.,?!:;/')):
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(4)
            for letter, symbol in zip(letter_row, symbol_row):
                key = self._button(letter, 'sketchKey', 0)
                key.setFocusPolicy(QtCore.Qt.NoFocus)
                row.addWidget(key, 1)
                keys.append(key)
                letters.append(letter)
                symbols.append(symbol)
                key.clicked.connect(lambda checked=False, key=key: self.canvas.type_text(key.text()))
            layout.addLayout(row)
        actions = QtWidgets.QHBoxLayout()
        actions.setSpacing(4)
        shift, symbol_button = self._button('Shift', 'sketchKey', 54), self._button('123', 'sketchKey', 54)
        space, back = self._button('Space', 'sketchKey', 100), self._button('Back', 'sketchKey', 54)
        enter, done = self._button('Enter', 'sketchKey', 54), self._button('Done', 'sketchKey', 54)
        shift.setCheckable(True)
        symbol_button.setCheckable(True)
        for button in (shift, symbol_button, space, back, enter, done):
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            actions.addWidget(button, 3 if button is space else 1)
        layout.addLayout(actions)

        def update_keys():
            for index, key in enumerate(keys):
                value = symbols[index] if symbol_button.isChecked() else letters[index]
                key.setText(value.upper() if shift.isChecked() and not symbol_button.isChecked() else value)
        shift.toggled.connect(update_keys)
        symbol_button.toggled.connect(update_keys)
        space.clicked.connect(lambda: self.canvas.type_text(' '))
        back.clicked.connect(self.canvas.backspace)
        enter.clicked.connect(lambda: self.canvas.type_text('\n'))
        done.clicked.connect(self.canvas.finish_editing)
        return keyboard

    def _update_buttons(self):
        self._undo.setEnabled(self.canvas._history_index > 0)
        self._redo.setEnabled(self.canvas._history_index + 1 < len(self.canvas._history))
        self._remove.setEnabled(bool(self.canvas.scene().selectedItems()))
        self._clear.setEnabled(bool(self.canvas.scene().items()))

    def _open(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Open sketch', _folder(), 'NOVA sketch (*.nova-sketch)')
        if path:
            try:
                self.canvas.load(path)
            except (OSError, ValueError) as error:
                QtWidgets.QMessageBox.warning(self, 'Could not open sketch', str(error))

    def _save(self):
        self._save_as(False)

    def _export(self):
        self._save_as(True)

    def _save_as(self, image):
        folder = _folder()
        try:
            Path(folder).mkdir(parents=True, exist_ok=True)
        except OSError as error:
            QtWidgets.QMessageBox.warning(self, 'Could not create folder', str(error))
            return
        suffix, label = ('.png', 'PNG image (*.png)') if image else ('.nova-sketch', 'NOVA sketch (*.nova-sketch)')
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Export image' if image else 'Save sketch',
                                                        str(Path(folder) / ('Sketch' + suffix)), label)
        if not path:
            return
        if not Path(path).suffix:
            path += suffix
        try:
            self.canvas.export_png(path) if image else self.canvas.save(path)
        except (OSError, ValueError) as error:
            QtWidgets.QMessageBox.warning(self, 'Could not export image' if image else 'Could not save sketch', str(error))
            return
        button = self.findChild(QtWidgets.QPushButton, 'sketchExport' if image else 'sketchSave')
        original = button.text()
        button.setText('Saved')
        QtCore.QTimer.singleShot(1500, lambda: button.setText(original))

    def showEvent(self, event):
        super().showEvent(event)
        self.canvas.scene().invalidate(self.canvas.sceneRect(), QtWidgets.QGraphicsScene.AllLayers)
        self.canvas.viewport().update()

    def finish_editing(self):
        self.canvas.finish_editing()

    def apply_theme(self):
        base = '''
            QWidget#sketchPage, QFrame#sketchToolbar, QFrame#sketchKeyboard { background: #000000; border: 0; }
            QLabel#sketchTitle { color: #f4f4f4; font-size: 18px; font-weight: 600; }
            QLabel#sketchHint { color: #aaaaaa; font-size: 13px; }
            QPushButton, QToolButton { color: #e6e6e6; background: #171717; border: 1px solid #3b3b3b; border-radius: 5px; padding: 0 6px; }
            QPushButton:hover, QToolButton:hover { background: #292929; border-color: #666666; }
            QPushButton:checked, QToolButton:checked { color: #111111; background: #ededed; border-color: #ededed; }
            QPushButton:disabled { color: #707070; background: #111111; border-color: #2c2c2c; }
            QPushButton#sketchKey { min-height: 34px; }
            QSlider::groove:horizontal { height: 4px; background: #555555; border-radius: 2px; }
            QSlider::handle:horizontal { width: 18px; margin: -8px 0; background: #ededed; border-radius: 9px; }
        '''
        light = '''
            QWidget#sketchPage, QFrame#sketchToolbar, QFrame#sketchKeyboard { background: #ffffff; }
            QLabel#sketchTitle { color: #202124; }
            QLabel#sketchHint { color: #585858; }
            QPushButton, QToolButton { color: #202124; background: #f3f3f3; border-color: #c7c7c7; }
            QPushButton:hover, QToolButton:hover { background: #e7e7e7; border-color: #999999; }
            QPushButton:checked, QToolButton:checked { color: #ffffff; background: #252525; border-color: #252525; }
            QPushButton:disabled { color: #8a8a8a; background: #f1f1f1; border-color: #dddddd; }
            QSlider::groove:horizontal { background: #b0b0b0; }
            QSlider::handle:horizontal { background: #262626; }
        '''
        self.setStyleSheet(base + light if _white_theme() else base)
        self.canvas.viewport().update()
