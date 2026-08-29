import time
from collections import deque

from python_qt_binding.QtCore import QPointF, QRectF, Qt
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from python_qt_binding.QtWidgets import QSizePolicy, QWidget


class TimeSeriesPlot(QWidget):
    def __init__(self, title, unit, series, window_seconds=20.0, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.series = series
        self.window_seconds = window_seconds
        self.history = {name: deque(maxlen=800) for name in series}
        self.setMinimumHeight(250)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def add_sample(self, values, timestamp=None):
        sample_time = time.monotonic() if timestamp is None else timestamp
        for name, value in values.items():
            if name in self.history:
                self.history[name].append((sample_time, float(value)))
        self.update()

    def clear(self):
        for samples in self.history.values():
            samples.clear()
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101722"))
        plot = QRectF(58, 42, max(10, self.width() - 78), max(10, self.height() - 82))
        now = time.monotonic()
        cutoff = now - self.window_seconds
        visible = {
            name: [(stamp, value) for stamp, value in samples if stamp >= cutoff]
            for name, samples in self.history.items()
        }
        all_values = [value for samples in visible.values() for _, value in samples]
        if all_values:
            minimum = min(min(all_values), 0.0)
            maximum = max(max(all_values), 0.0)
        else:
            minimum, maximum = -1.0, 1.0
        span = maximum - minimum
        if span < 1.0e-6:
            span = max(abs(maximum), 1.0) * 0.2
            minimum -= span
            maximum += span
        else:
            margin = 0.1 * span
            minimum -= margin
            maximum += margin

        painter.setFont(QFont("Noto Sans CJK SC", 10, QFont.Bold))
        painter.setPen(QColor("#e2e8f0"))
        painter.drawText(QRectF(12, 8, self.width() - 24, 24), Qt.AlignLeft, self.title)
        painter.setFont(QFont("Noto Sans CJK SC", 8))
        painter.setPen(QPen(QColor("#26364b"), 1))
        for index in range(5):
            ratio = index / 4.0
            y = plot.top() + ratio * plot.height()
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            value = maximum - ratio * (maximum - minimum)
            painter.setPen(QColor("#94a3b8"))
            painter.drawText(QRectF(2, y - 9, 52, 18), Qt.AlignRight, f"{value:+.2f}")
            painter.setPen(QPen(QColor("#26364b"), 1))
        painter.setPen(QColor("#64748b"))
        painter.drawText(QRectF(plot.left(), plot.bottom() + 6, 80, 18), Qt.AlignLeft, "-20 s")
        painter.drawText(QRectF(plot.right() - 60, plot.bottom() + 6, 60, 18), Qt.AlignRight, "现在")
        painter.drawText(QRectF(plot.left(), 8, plot.width(), 24), Qt.AlignRight, self.unit)

        for name, color in self.series.items():
            points = QPolygonF()
            for stamp, value in visible[name]:
                x = plot.right() - (now - stamp) / self.window_seconds * plot.width()
                y = plot.bottom() - (value - minimum) / (maximum - minimum) * plot.height()
                points.append(QPointF(x, y))
            if len(points) >= 2:
                painter.setPen(QPen(QColor(color), 2))
                painter.drawPolyline(points)

        legend_x = plot.left()
        for name, color in self.series.items():
            painter.setPen(QPen(QColor(color), 3))
            painter.drawLine(QPointF(legend_x, 33), QPointF(legend_x + 16, 33))
            painter.setPen(QColor("#cbd5e1"))
            painter.drawText(QRectF(legend_x + 20, 24, 78, 18), Qt.AlignLeft, name)
            legend_x += 98
