import sys
import heapq
from collections import defaultdict
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QGraphicsView, QGraphicsScene,
    QGraphicsItem, QGraphicsRectItem, QGraphicsPathItem,
    QToolBar, QStatusBar, QLabel
)
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import (
    QPen, QBrush, QColor, QPainter, QPainterPath, QFont, QTransform
)

# ── Constants ────────────────────────────────────────────────────────────────

WIRE_COLOR   = QColor(0, 0, 0)
WIRE_SEL_COLOR = QColor(200, 0, 0)
GRID_MINOR   = 20
GRID_MAJOR   = 100
SNAP_GRID    = GRID_MINOR

CELL_SIZE          = 8
COMP_PAD_CELLS     = 1
PIN_EXIT_CELLS     = 2
TURN_PENALTY       = 4
CONGESTION_PENALTY = 30


# ── A* Router ─────────────────────────────────────────────────────────────────

class RouterGrid:
    def __init__(self, components: list):
        if not components:
            self.cols = self.rows = 1
            self.ox = self.oy = 0
            self._blocked: set[tuple[int, int]] = set()
            self._congestion: dict[tuple[int, int], int] = {}
            return

        pad_px = COMP_PAD_CELLS * CELL_SIZE
        margin  = PIN_EXIT_CELLS * CELL_SIZE * 4

        min_x = min(c.pos().x() for c in components) - pad_px - margin
        min_y = min(c.pos().y() for c in components) - pad_px - margin
        max_x = max(c.pos().x() + c.w for c in components) + pad_px + margin
        max_y = max(c.pos().y() + c.h for c in components) + pad_px + margin

        self.ox   = min_x
        self.oy   = min_y
        self.cols = max(1, int((max_x - min_x) / CELL_SIZE) + 2)
        self.rows = max(1, int((max_y - min_y) / CELL_SIZE) + 2)
        self._blocked: set[tuple[int, int]] = set()
        self._congestion: dict[tuple[int, int], int] = {}

        for comp in components:
            px, py = comp.pos().x(), comp.pos().y()
            c0 = max(0, self._to_col(px - pad_px))
            r0 = max(0, self._to_row(py - pad_px))
            c1 = min(self.cols - 1, self._to_col(px + comp.w + pad_px))
            r1 = min(self.rows - 1, self._to_row(py + comp.h + pad_px))
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    self._blocked.add((r, c))

    def _to_col(self, x): return int((x - self.ox) / CELL_SIZE)
    def _to_row(self, y): return int((y - self.oy) / CELL_SIZE)

    def world_to_cell(self, x, y): return self._to_row(y), self._to_col(x)

    def cell_to_world(self, r, c):
        return (c * CELL_SIZE + self.ox + CELL_SIZE / 2,
                r * CELL_SIZE + self.oy + CELL_SIZE / 2)

    def is_blocked(self, r, c, exclude=None):
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols:
            return True
        if exclude and (r, c) in exclude:
            return False
        return (r, c) in self._blocked

    def congestion_cost(self, r, c):
        return self._congestion.get((r, c), 0) * CONGESTION_PENALTY

    def mark_cell(self, r, c):
        self._congestion[(r, c)] = self._congestion.get((r, c), 0) + 1


_DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def astar_route(grid, start: QPointF, end: QPointF):
    sr, sc = grid.world_to_cell(start.x(), start.y())
    er, ec = grid.world_to_cell(end.x(), end.y())
    if (sr, sc) == (er, ec):
        return [start, end]
    exclude = frozenset({(sr, sc), (er, ec)})

    def h(r, c): return abs(r - er) + abs(c - ec)

    heap = [(h(sr, sc), 0, sr, sc, -1)]
    g_scores = {(sr, sc, -1): 0}
    came_from = {}

    while heap:
        f, g, r, c, d = heapq.heappop(heap)
        state = (r, c, d)
        if g > g_scores.get(state, 10**9):
            continue
        if r == er and c == ec:
            cells = []
            cur = state
            while cur in came_from:
                cells.append(cur[:2])
                cur = came_from[cur]
            cells.append((sr, sc))
            cells.reverse()
            return [QPointF(*grid.cell_to_world(pr, pc)) for pr, pc in cells]
        for di, (dr, dc) in enumerate(_DIRS):
            nr, nc = r + dr, c + dc
            if grid.is_blocked(nr, nc, exclude):
                continue
            turn_cost = TURN_PENALTY if (d != -1 and di != d) else 0
            ng = g + 1 + turn_cost + grid.congestion_cost(nr, nc)
            nstate = (nr, nc, di)
            if ng < g_scores.get(nstate, 10**9):
                g_scores[nstate] = ng
                came_from[nstate] = state
                heapq.heappush(heap, (ng + h(nr, nc), ng, nr, nc, di))
    return None


def simplify_path(pts):
    if len(pts) < 3:
        return pts
    result = [pts[0]]
    for i in range(1, len(pts) - 1):
        prev, curr, nxt = result[-1], pts[i], pts[i + 1]
        same_x = abs(prev.x() - curr.x()) < 0.5 and abs(curr.x() - nxt.x()) < 0.5
        same_y = abs(prev.y() - curr.y()) < 0.5 and abs(curr.y() - nxt.y()) < 0.5
        if not (same_x or same_y):
            result.append(curr)
    result.append(pts[-1])
    return result


def pin_exit_point(pin) -> QPointF:
    p   = pin.scene_pos()
    off = PIN_EXIT_CELLS * CELL_SIZE
    return {
        'right':  QPointF(p.x() + off, p.y()),
        'left':   QPointF(p.x() - off, p.y()),
        'bottom': QPointF(p.x(), p.y() + off),
        'top':    QPointF(p.x(), p.y() - off),
    }.get(pin.side, p)


def _seg_crosses_rect(a: QPointF, b: QPointF, rect: QRectF) -> bool:
    """Return True if segment a→b intersects or is inside rect."""
    if rect.contains(a) or rect.contains(b):
        return True
    # Cohen-Sutherland clip: if segment clips into rect it intersects
    LEFT, RIGHT, BOTTOM, TOP = 1, 2, 4, 8

    def code(p):
        c = 0
        if p.x() < rect.left():   c |= LEFT
        elif p.x() > rect.right(): c |= RIGHT
        if p.y() < rect.top():    c |= TOP
        elif p.y() > rect.bottom(): c |= BOTTOM
        return c

    c1, c2 = code(a), code(b)
    if c1 & c2:        # both on same side — trivially outside
        return False
    if c1 == 0 or c2 == 0:
        return True    # one inside
    # At least one edge may be crossed — test all 4 rect edges with cross product
    def cross_z(o, u, v):
        return (u.x()-o.x())*(v.y()-o.y()) - (u.y()-o.y())*(v.x()-o.x())
    corners = [
        (QPointF(rect.left(),  rect.top()),    QPointF(rect.right(), rect.top())),
        (QPointF(rect.right(), rect.top()),    QPointF(rect.right(), rect.bottom())),
        (QPointF(rect.right(), rect.bottom()), QPointF(rect.left(),  rect.bottom())),
        (QPointF(rect.left(),  rect.bottom()), QPointF(rect.left(),  rect.top())),
    ]
    for e0, e1 in corners:
        if (cross_z(e0, e1, a) * cross_z(e0, e1, b) < 0 and
                cross_z(a, b, e0) * cross_z(a, b, e1) < 0):
            return True
    return False


def _mark_path(grid, p_start, waypoints, p_end):
    pts = [p_start] + waypoints + [p_end]
    for i in range(len(pts) - 1):
        r1, c1 = grid.world_to_cell(pts[i].x(), pts[i].y())
        r2, c2 = grid.world_to_cell(pts[i+1].x(), pts[i+1].y())
        steps = max(abs(r2 - r1), abs(c2 - c1))
        if steps == 0:
            grid.mark_cell(r1, c1)
            continue
        for t in range(steps + 1):
            r = r1 + round((r2 - r1) * t / steps)
            c = c1 + round((c2 - c1) * t / steps)
            grid.mark_cell(r, c)


# ── Pin ──────────────────────────────────────────────────────────────────────

class PinItem(QGraphicsRectItem):
    SIZE = 7

    def __init__(self, pin_id, side, parent_component):
        s = self.SIZE
        super().__init__(-s / 2, -s / 2, s, s)
        self.pin_id = pin_id
        self.side   = side
        self.parent_component = parent_component
        self.connections = []
        self.setPen(QPen(QColor(30, 30, 30), 1.2))
        self.setBrush(QBrush(QColor(240, 220, 40)))
        self.setZValue(3)

    def scene_pos(self) -> QPointF:
        return self.mapToScene(self.rect().center())

    def paint(self, painter, option, widget=None):
        super().paint(painter, option, widget)
        painter.setFont(QFont("Arial", 5))
        painter.setPen(QPen(QColor(10, 10, 10)))
        painter.drawText(self.boundingRect(), Qt.AlignmentFlag.AlignCenter,
                         str(self.pin_id))


# ── Component Base ────────────────────────────────────────────────────────────

class ComponentItem(QGraphicsItem):
    MIN_W, MAX_W = 90, 160
    MIN_H, MAX_H = 70, 130

    def __init__(self, comp_id, comp_type, color):
        super().__init__()
        self.comp_id   = comp_id
        self.comp_type = comp_type
        self.color     = color
        self.w = 120
        self.h = 100
        self.pins = []
        self._init_flags()

    def _init_flags(self):
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.setZValue(1)

    def boundingRect(self):
        return QRectF(-4, -4, self.w + 8, self.h + 8)

    def _draw_body(self, painter):
        w, h = self.w, self.h
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 45)))
        painter.drawRoundedRect(QRectF(5, 5, w, h), 6, 6)
        painter.setBrush(QBrush(self.color))
        border = (QPen(QColor(255, 165, 0), 2.5) if self.isSelected()
                  else QPen(QColor(30, 30, 30), 1.5))
        painter.setPen(border)
        painter.drawRoundedRect(QRectF(0, 0, w, h), 6, 6)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(self.color.darker(155)))
        painter.drawRoundedRect(QRectF(0, 0, w, 22), 6, 6)
        painter.drawRect(QRectF(0, 16, w, 6))
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        painter.drawText(QRectF(3, 3, w - 6, 18), Qt.AlignmentFlag.AlignCenter,
                         self.comp_type)
        painter.setPen(QPen(QColor(220, 220, 220)))
        painter.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        painter.drawText(QRectF(0, 22, w, h - 22),
                         Qt.AlignmentFlag.AlignCenter, f"U{self.comp_id}")

    def paint(self, painter, option, widget=None):
        self._draw_body(painter)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            x = round(value.x() / SNAP_GRID) * SNAP_GRID
            y = round(value.y() / SNAP_GRID) * SNAP_GRID
            return QPointF(x, y)
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for pin in self.pins:
                for conn in pin.connections:
                    conn.update_path()
            scene = self.scene()
            if scene is not None:
                scene._dirty_components.add(self)
                scene._needs_reroute = True
        return super().itemChange(change, value)


# ── RF Component (named pins) ─────────────────────────────────────────────────

class RFComponentItem(ComponentItem):
    """Component with explicitly named pins. pin_spec = [(side, name), ...]"""

    def __init__(self, comp_id, comp_type, color, w, h, pin_spec):
        super().__init__(comp_id, comp_type, color)
        self.w = w
        self.h = h
        self.pin_names: dict[int, str] = {}
        self._build_named_pins(pin_spec)

    def _build_named_pins(self, pin_spec):
        by_side = defaultdict(list)
        for side, name in pin_spec:
            by_side[side].append(name)
        pid = 0
        for side in ('left', 'right', 'top', 'bottom'):
            names = by_side[side]
            count = len(names)
            for i, name in enumerate(names):
                f = (i + 1) / (count + 1)
                if side == 'left':   x, y = 0,       self.h * f
                elif side == 'right': x, y = self.w,  self.h * f
                elif side == 'top':   x, y = self.w * f, 0
                else:                 x, y = self.w * f, self.h
                pin = PinItem(pid, side, self)
                pin.setParentItem(self)
                pin.setPos(x, y)
                self.pins.append(pin)
                self.pin_names[pid] = name
                pid += 1

    def get_pin(self, name: str):
        for pid, pname in self.pin_names.items():
            if pname == name:
                return self.pins[pid]
        return None

    def paint(self, painter, option, widget=None):
        self._draw_body(painter)
        painter.setFont(QFont("Arial", 4))
        half = self.w // 2
        for pin in self.pins:
            name = self.pin_names.get(pin.pin_id, "")
            px, py = pin.pos().x(), pin.pos().y()
            if pin.side == 'left':
                painter.setPen(QPen(QColor(210, 210, 210)))
                painter.drawText(QRectF(px + 5, py - 4, half - 5, 8),
                                 Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                                 name)
            elif pin.side == 'right':
                painter.setPen(QPen(QColor(210, 210, 210)))
                painter.drawText(QRectF(half, py - 4, half - 5, 8),
                                 Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                                 name)


# ── RFIC ─────────────────────────────────────────────────────────────────────

_RFIC_PINS = (
    [('left',   n) for n in ['RX1_HB','RX2_HB','RX1_MB','RX2_MB','RX_LB',
                               'ANT_TUNE','VCC','GND','SPI_CLK','SPI_DATA']] +
    [('right',  n) for n in ['TX_HB','TX_MB','TX_LB','PA_EN','LNA_EN',
                               'CPL_IN','BIAS','VCC2','RESETN','GPIO']] +
    [('top',    n) for n in ['RFFE_CLK','RFFE_DATA','VCC_IO']] +
    [('bottom', n) for n in ['GND_RF','VCC_RF','GNDD']]
)


class RFICItem(RFComponentItem):
    def __init__(self, comp_id):
        super().__init__(comp_id, "RFIC-5G", QColor(178, 68, 50),
                         w=160, h=300, pin_spec=_RFIC_PINS)

    def paint(self, painter, option, widget=None):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.w, self.h
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 45)))
        painter.drawRoundedRect(QRectF(5, 5, w, h), 6, 6)
        painter.setBrush(QBrush(self.color))
        border = (QPen(QColor(255, 165, 0), 2.5) if self.isSelected()
                  else QPen(QColor(20, 20, 20), 1.8))
        painter.setPen(border)
        painter.drawRoundedRect(QRectF(0, 0, w, h), 6, 6)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(self.color.darker(160)))
        painter.drawRoundedRect(QRectF(0, 0, w, 24), 6, 6)
        painter.drawRect(QRectF(0, 18, w, 6))
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        painter.drawText(QRectF(3, 3, w - 6, 20), Qt.AlignmentFlag.AlignCenter,
                         self.comp_type)
        painter.setPen(QPen(QColor(220, 255, 220)))
        painter.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
        painter.drawText(QRectF(0, 28, w, h - 50),
                         Qt.AlignmentFlag.AlignCenter, f"U{self.comp_id}")
        # pin name labels
        painter.setFont(QFont("Arial", 4))
        half = w // 2
        for pin in self.pins:
            name = self.pin_names.get(pin.pin_id, "")
            px, py = pin.pos().x(), pin.pos().y()
            if pin.side == 'left':
                painter.setPen(QPen(QColor(180, 230, 180)))
                painter.drawText(QRectF(px + 5, py - 4, half - 5, 8),
                                 Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                                 name)
            elif pin.side == 'right':
                painter.setPen(QPen(QColor(180, 230, 180)))
                painter.drawText(QRectF(half, py - 4, half - 5, 8),
                                 Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                                 name)
        painter.setPen(QPen(QColor(160, 230, 160)))
        painter.setFont(QFont("Arial", 6))
        painter.drawText(QRectF(2, h - 18, w - 4, 14),
                         Qt.AlignmentFlag.AlignCenter, f"{len(self.pins)} pins")


# ── Antenna ───────────────────────────────────────────────────────────────────

class AntennaItem(QGraphicsItem):
    def __init__(self, comp_id):
        super().__init__()
        self.comp_id   = comp_id
        self.comp_type = "ANT"
        self.w = 80
        self.h = 140
        self.pins = []
        self.pin_names: dict[int, str] = {}
        pin = PinItem(0, 'left', self)
        pin.setParentItem(self)
        pin.setPos(0, self.h // 2)
        self.pins.append(pin)
        self.pin_names[0] = "FEED"
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.setZValue(1)

    def get_pin(self, name):
        for pid, pname in self.pin_names.items():
            if pname == name:
                return self.pins[pid]
        return None

    def boundingRect(self):
        return QRectF(-4, -4, self.w + 8, self.h + 8)

    def paint(self, painter, option, widget=None):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.w, self.h
        cx = w * 0.55
        cy = h / 2

        # Feed line
        painter.setPen(QPen(QColor(40, 40, 40), 2))
        painter.drawLine(QPointF(0, cy), QPointF(cx * 0.5, cy))

        # Cone (antenna symbol)
        tip_x  = cx * 0.5
        base_x = w * 0.95
        half_h = h * 0.38
        path = QPainterPath()
        path.moveTo(tip_x, cy)
        path.lineTo(base_x, cy - half_h)
        path.lineTo(base_x, cy + half_h)
        path.closeSubpath()

        col = QColor(180, 100, 20) if not self.isSelected() else QColor(255, 165, 0)
        painter.setBrush(QBrush(col))
        painter.setPen(QPen(QColor(80, 40, 0), 1.5))
        painter.drawPath(path)

        # Ground bars at feed end (left)
        painter.setPen(QPen(QColor(40, 40, 40), 1.5))
        for i, half_w in enumerate([10, 7, 4]):
            yy = cy + 8 + i * 5
            painter.drawLine(QPointF(0 - half_w, yy), QPointF(0 + half_w, yy))

        # Label
        painter.setPen(QPen(QColor(30, 30, 30)))
        painter.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        painter.drawText(QRectF(cx * 0.3, cy - h * 0.45, w * 0.6, 14),
                         Qt.AlignmentFlag.AlignCenter, "ANT")

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            x = round(value.x() / SNAP_GRID) * SNAP_GRID
            y = round(value.y() / SNAP_GRID) * SNAP_GRID
            return QPointF(x, y)
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for pin in self.pins:
                for conn in pin.connections:
                    conn.update_path()
            scene = self.scene()
            if scene is not None:
                scene._dirty_components.add(self)
                scene._needs_reroute = True
        return super().itemChange(change, value)


# ── Connection ────────────────────────────────────────────────────────────────

class ConnectionItem(QGraphicsPathItem):
    def __init__(self, pin1: PinItem, pin2: PinItem, label: str = ""):
        super().__init__()
        self.pin1  = pin1
        self.pin2  = pin2
        self.label = label
        self._waypoints = None
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setZValue(0)
        self._draw_bezier()

    def set_routed_path(self, waypoints):
        self._waypoints = waypoints
        self._draw_routed()

    def update_path(self):
        if self._waypoints:
            self._draw_routed()
        else:
            self._draw_bezier()

    def _draw_routed(self):
        # self.pin1.rect().center()
        p1 = self.pin1.scene_pos()
        p2 = self.pin2.scene_pos()
        wpts = list(self._waypoints) if self._waypoints else []

        # Fix small bend: snap first waypoint's transverse axis to match pin position
        if wpts:
            if self.pin1.side in ('left', 'right'):
                wpts[0] = QPointF(wpts[0].x(), p1.y())
            else:
                wpts[0] = QPointF(p1.x(), wpts[0].y())

        if len(wpts) >= 2:
            if self.pin2.side in ('left', 'right'):
                wpts[-1] = QPointF(wpts[-1].x(), p2.y())
            else:
                wpts[-1] = QPointF(p2.x(), wpts[-1].y())

        path = QPainterPath(p1)
        for pt in wpts:
            path.lineTo(pt)
        path.lineTo(p2)
        self.setPath(path)

    def _draw_bezier(self):
        p1 = self.pin1.scene_pos()
        p2 = self.pin2.scene_pos()
        dist = max(abs(p2.x() - p1.x()), abs(p2.y() - p1.y())) * 0.45 + 40
        offsets = {'right': (dist, 0), 'left': (-dist, 0),
                   'bottom': (0, dist), 'top': (0, -dist)}
        ox1, oy1 = offsets.get(self.pin1.side, (0, 0))
        ox2, oy2 = offsets.get(self.pin2.side, (0, 0))
        path = QPainterPath(p1)
        path.cubicTo(QPointF(p1.x() + ox1, p1.y() + oy1),
                     QPointF(p2.x() + ox2, p2.y() + oy2), p2)
        self.setPath(path)

    def paint(self, painter, option, widget=None):
        if self.isSelected():
            pen = QPen(WIRE_SEL_COLOR, 2.0)
            pen.setStyle(Qt.PenStyle.DashDotLine)
            pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        else:
            pen = QPen(WIRE_COLOR, 1.5)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.MiterJoin)
        painter.setPen(pen)
        painter.drawPath(self.path())

        # Net label near midpoint
        if self.label:
            pts = self.path()
            mid = pts.pointAtPercent(0.5)
            painter.setFont(QFont("Arial", 5))
            painter.setPen(QPen(QColor(80, 80, 200)))
            painter.drawText(mid + QPointF(3, -3), self.label)


# ── Scene ─────────────────────────────────────────────────────────────────────

class CADScene(QGraphicsScene):

    def __init__(self):
        super().__init__()
        self.setSceneRect(-300, -200, 2800, 1400)
        self.components:  list = []
        self.connections: list[ConnectionItem] = []
        self._needs_reroute = False
        self._dirty_components: set = set()
        self._populate()
        self.route_all()

    # ── helpers ───────────────────────────────────────────────────────────────

    def _add_comp(self, comp):
        self.addItem(comp)
        self.components.append(comp)
        return comp

    def _connect(self, pin1: PinItem, pin2: PinItem, label: str = ""):
        if pin1 is None or pin2 is None:
            return
        conn = ConnectionItem(pin1, pin2, label)
        self.addItem(conn)
        pin1.connections.append(conn)
        pin2.connections.append(conn)
        self.connections.append(conn)

    # ── S23-style RF Frontend schematic ──────────────────────────────────────

    def _populate(self):
        # ── Components ──────────────────────────────────────────────────────

        # RFIC — center
        rfic = self._add_comp(RFICItem(1))
        rfic.setPos(620, 160)

        # LNA — top-left
        lna = self._add_comp(RFComponentItem(
            2, "LNA", QColor(70, 160, 210), w=130, h=110,
            pin_spec=[
                ('left',  'RF_IN'), ('left',  'VCC'), ('left',  'GND'),
                ('right', 'RF_OUT'), ('right', 'EN'), ('right', 'BYPASS'),
            ]
        ))
        lna.setPos(160, 40)

        # PAM — top-right
        pam = self._add_comp(RFComponentItem(
            3, "PAM", QColor(70, 160, 210), w=140, h=130,
            pin_spec=[
                ('left',  'RF_IN_HB'), ('left',  'RF_IN_MB'),
                ('left',  'VCC_PA'),   ('left',  'GND'),
                ('right', 'RF_OUT_HB'), ('right', 'RF_OUT_MB'),
                ('right', 'PA_EN'),    ('right', 'VCTL'),
            ]
        ))
        pam.setPos(1000, 40)

        # LPAMID — bottom-right
        lpamid = self._add_comp(RFComponentItem(
            4, "LPAMID", QColor(70, 160, 210), w=130, h=100,
            pin_spec=[
                ('left',  'RF_IN'), ('left',  'VCC'), ('left',  'GND'),
                ('right', 'RF_OUT'), ('right', 'EN'), ('right', 'VCTL'),
            ]
        ))
        lpamid.setPos(1000, 400)

        # Coupler — far right top
        coupler = self._add_comp(RFComponentItem(
            5, "Coupler", QColor(70, 160, 210), w=110, h=90,
            pin_spec=[
                ('left',  'IN'),      ('left',  'COUPLED'),
                ('right', 'THRU'),    ('right', 'ISO'),
            ]
        ))
        coupler.setPos(1260, 40)

        # FEM — bottom-left
        fem = self._add_comp(RFComponentItem(
            6, "FEM", QColor(70, 160, 210), w=140, h=130,
            pin_spec=[
                ('left',  'ANT_PORT'), ('left',  'TX_IN'),
                ('left',  'VCC'),      ('left',  'GND'),
                ('right', 'RX_OUT'),   ('right', 'TX_OUT'),
                ('right', 'RX_IN'),    ('right', 'CTRL'),
            ]
        ))
        fem.setPos(160, 420)

        # Switch — far right center
        switch = self._add_comp(RFComponentItem(
            7, "Switch", QColor(70, 160, 210), w=120, h=130,
            pin_spec=[
                ('left',  'ANT'),
                ('right', 'TX1'), ('right', 'TX2'),
                ('right', 'RX1'), ('right', 'RX2'),
            ]
        ))
        switch.setPos(1260, 280)

        # Antenna — far right
        ant = self._add_comp(AntennaItem(8))
        ant.setPos(1500, 220)

        # ── Connections ─────────────────────────────────────────────────────

        # TX high-band: RFIC → PAM → Coupler → Switch(TX1)
        self._connect(rfic.get_pin('TX_HB'),      pam.get_pin('RF_IN_HB'),  'TX_HB')
        self._connect(pam.get_pin('RF_OUT_HB'),   coupler.get_pin('IN'),    'PA_OUT_HB')
        self._connect(coupler.get_pin('THRU'),    switch.get_pin('TX1'),    'TX1_THRU')
        self._connect(coupler.get_pin('COUPLED'), rfic.get_pin('CPL_IN'),   'CPL_FB')

        # TX mid-band: RFIC → PAM → Switch(TX2)
        self._connect(rfic.get_pin('TX_MB'),      pam.get_pin('RF_IN_MB'),  'TX_MB')
        self._connect(pam.get_pin('RF_OUT_MB'),   switch.get_pin('TX2'),    'PA_OUT_MB')

        # TX low-band: RFIC → LPAMID → FEM
        self._connect(rfic.get_pin('TX_LB'),      lpamid.get_pin('RF_IN'), 'TX_LB')
        self._connect(lpamid.get_pin('RF_OUT'),   fem.get_pin('TX_IN'),    'TX_LB_PA')

        # RX high-band: Switch(RX1) → LNA → RFIC
        self._connect(switch.get_pin('RX1'),      lna.get_pin('RF_IN'),    'RX_HB')
        self._connect(lna.get_pin('RF_OUT'),      rfic.get_pin('RX1_HB'),  'LNA_OUT_HB')

        # RX low-band: Switch(RX2) → FEM → RFIC
        self._connect(switch.get_pin('RX2'),      fem.get_pin('ANT_PORT'), 'RX_LB_ANT')
        self._connect(fem.get_pin('RX_OUT'),      rfic.get_pin('RX_LB'),   'FEM_RX_OUT')

        # Antenna to switch
        self._connect(ant.get_pin('FEED'),        switch.get_pin('ANT'),   'ANT_PORT')

        # Control lines
        self._connect(rfic.get_pin('LNA_EN'),     lna.get_pin('EN'),       'LNA_EN')
        self._connect(rfic.get_pin('PA_EN'),      pam.get_pin('PA_EN'),    'PA_EN')

    # ── A* routing ────────────────────────────────────────────────────────────

    def route_all(self, partial: bool = False):
        """
        Route connections using A*.
        partial=True: only re-route connections touching dirty (moved) components;
                      all other connections keep their cached paths and are
                      replayed onto the grid so congestion stays accurate.
        partial=False (default / "Re-route All" button): re-route everything.
        """
        dirty = self._dirty_components if partial else set()
        self._needs_reroute = False
        self._dirty_components = set()

        grid = RouterGrid(self.components)

        def net_len(conn):
            p1, p2 = conn.pin1.scene_pos(), conn.pin2.scene_pos()
            return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y())

        affected = set()
        if partial and dirty:
            # 1. Connections directly attached to a moved component
            for comp in dirty:
                for pin in comp.pins:
                    for conn in pin.connections:
                        affected.add(conn)

            # 2. Connections whose cached path passes through the moved component's
            #    bounding rect — they may now be blocked by the component's new position
            pad = (COMP_PAD_CELLS + 1) * CELL_SIZE
            dirty_rects = [
                QRectF(c.pos().x() - pad, c.pos().y() - pad,
                       c.w + 2 * pad, c.h + 2 * pad)
                for c in dirty
            ]
            for conn in self.connections:
                if conn in affected or not conn._waypoints:
                    continue
                all_pts = ([conn.pin1.scene_pos()] +
                           list(conn._waypoints) +
                           [conn.pin2.scene_pos()])
                for i in range(len(all_pts) - 1):
                    if any(_seg_crosses_rect(all_pts[i], all_pts[i+1], r)
                           for r in dirty_rects):
                        affected.add(conn)
                        break

        # Replay cached paths for untouched connections first (congestion map)
        stable = [c for c in self.connections if c not in affected]
        for conn in stable:
            if conn._waypoints:
                _mark_path(grid, conn.pin1.scene_pos(), conn._waypoints,
                           conn.pin2.scene_pos())

        # Re-route only the affected (or all) connections
        to_route = sorted(affected if partial and dirty else self.connections,
                          key=net_len)
        routed = fallback = 0
        for conn in to_route:
            path = astar_route(grid, pin_exit_point(conn.pin1),
                               pin_exit_point(conn.pin2))
            if path:
                simplified = simplify_path(path)
                conn.set_routed_path(simplified)
                _mark_path(grid, conn.pin1.scene_pos(), simplified,
                           conn.pin2.scene_pos())
                routed += 1
            else:
                conn.update_path()
                fallback += 1
        return routed, fallback

    # ── background ────────────────────────────────────────────────────────────

    def drawBackground(self, painter, rect):
        painter.fillRect(rect, QColor(255, 255, 255))
        # return
        # Draw coarse grids first, then fine cell grid on top
        for step, color, width in [
            (GRID_MAJOR, QColor(160, 160, 175), 1.2),
            # (GRID_MINOR, QColor(195, 195, 205), 0.8),
            # (GRID_MAJOR, QColor(0, 0, 0), 1.2),
            # (GRID_MINOR, QColor(0, 0, 0), 0.8),
        ]:
            painter.setPen(QPen(color, width))
            left = int(rect.left()) - int(rect.left()) % step
            top  = int(rect.top())  - int(rect.top())  % step
            x = left
            while x <= rect.right():
                painter.drawLine(x, int(rect.top()), x, int(rect.bottom()))
                x += step
            y = top
            while y <= rect.bottom():
                painter.drawLine(int(rect.left()), y, int(rect.right()), y)
                y += step

        # Cell-size grid on top — blue-tinted so it's distinct from coarse grids
        cell_pen = QPen(QColor(210, 215, 230), 1.0)
        painter.setPen(cell_pen)
        left = int(rect.left()) - int(rect.left()) % CELL_SIZE
        top  = int(rect.top())  - int(rect.top())  % CELL_SIZE
        x = left
        while x <= rect.right():
            painter.drawLine(x, int(rect.top()), x, int(rect.bottom()))
            x += CELL_SIZE
        y = top
        while y <= rect.bottom():
            painter.drawLine(int(rect.left()), y, int(rect.right()), y)
            y += CELL_SIZE


# ── View ──────────────────────────────────────────────────────────────────────

class CADView(QGraphicsView):
    ZOOM_FACTOR = 1.18

    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.SmartViewportUpdate)
        self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self._panning = False
        self._pan_origin = QPointF()

    def wheelEvent(self, event):
        f = self.ZOOM_FACTOR if event.angleDelta().y() > 0 else 1 / self.ZOOM_FACTOR
        self.scale(f, f)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_origin = event.pos()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning:
            d = event.pos() - self._pan_origin
            self._pan_origin = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - d.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - d.y())
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = False
            self.setCursor(Qt.CursorShape.ArrowCursor)
        else:
            super().mouseReleaseEvent(event)
            if event.button() == Qt.MouseButton.LeftButton:
                s = self.scene()
                if getattr(s, '_needs_reroute', False):
                    s.route_all(partial=True)


# ── Main Window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RF Frontend Schematic — Samsung S23 Style")
        self.resize(1400, 900)
        self.cad_scene = CADScene()
        self.cad_view  = CADView(self.cad_scene)
        self.setCentralWidget(self.cad_view)
        self._build_toolbar()
        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._refresh_status()
        self.fit_all()

    def _build_toolbar(self):
        tb = QToolBar("Tools")
        tb.setMovable(False)
        self.addToolBar(tb)
        tb.addAction("Fit All",      self.fit_all)
        tb.addAction("Zoom 1:1",     self.zoom_reset)
        tb.addSeparator()
        tb.addAction("Re-route All", self.reroute)
        tb.addAction("Reload",       self.reload)
        tb.addSeparator()
        tb.addWidget(QLabel(
            "  Scroll=Zoom  |  Middle-drag=Pan  |  Click=Select wire/comp  "
            "|  Drag=Move (snaps grid, auto-reroutes)"))

    def _refresh_status(self, routed=None, fallback=None):
        n_comp = len(self.cad_scene.components)
        n_conn = len(self.cad_scene.connections)
        extra  = (f"   |   A* routed: {routed}   Bezier fallback: {fallback}"
                  if routed is not None else "")
        self._status.showMessage(
            f"Components: {n_comp}   Connections: {n_conn}{extra}")

    def fit_all(self):
        self.cad_view.fitInView(self.cad_scene.itemsBoundingRect(),
                                Qt.AspectRatioMode.KeepAspectRatio)

    def zoom_reset(self):
        self.cad_view.setTransform(QTransform())

    def reroute(self):
        routed, fallback = self.cad_scene.route_all()
        self._refresh_status(routed, fallback)

    def reload(self):
        self.cad_scene = CADScene()
        self.cad_view.setScene(self.cad_scene)
        self._refresh_status()
        self.fit_all()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
