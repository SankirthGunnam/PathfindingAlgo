import sys
import random
import heapq
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

COMPONENT_TYPES = [
    "Resistor", "Capacitor", "Inductor", "IC-74HC", "Transistor",
    "Diode", "Op-Amp", "MCU-STM32", "FPGA", "DRAM", "EEPROM", "ADC",
    "DAC", "LDO Reg", "Switch", "Gyroscope", "LED Driver", "Crystal",
    "Transformer", "Relay"
]

COMPONENT_COLORS = [
    QColor(52, 101, 164),
    QColor(39, 139, 82),
    QColor(196, 100, 18),
    QColor(117, 80, 173),
    QColor(180, 60, 60),
    QColor(30, 160, 150),
    QColor(160, 140, 20),
    QColor(90, 110, 130),
]

WIRE_COLOR   = QColor(0, 0, 0)   # all connections are black
GRID_MINOR   = 20
GRID_MAJOR   = 100
SNAP_GRID    = GRID_MINOR

# ── A* Router ─────────────────────────────────────────────────────────────────

CELL_SIZE          = 8
COMP_PAD_CELLS     = 1
PIN_EXIT_CELLS     = 3
TURN_PENALTY       = 4    # cost for changing direction
CONGESTION_PENALTY = 30   # extra cost per wire already occupying a cell


class RouterGrid:
    """Rasterised obstacle + congestion map built from component positions."""

    def __init__(self, components: list):
        if not components:
            self.cols = self.rows = 1
            self.ox = self.oy = 0
            self._blocked: set[tuple[int, int]] = set()
            self._congestion: dict[tuple[int, int], int] = {}
            return

        pad_px = COMP_PAD_CELLS * CELL_SIZE
        margin = PIN_EXIT_CELLS * CELL_SIZE * 4

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

    # ── coordinate helpers ────────────────────────────────────────────────────

    def _to_col(self, x: float) -> int:
        return int((x - self.ox) / CELL_SIZE)

    def _to_row(self, y: float) -> int:
        return int((y - self.oy) / CELL_SIZE)

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return self._to_row(y), self._to_col(x)

    def cell_to_world(self, r: int, c: int) -> tuple[float, float]:
        return (c * CELL_SIZE + self.ox + CELL_SIZE / 2,
                r * CELL_SIZE + self.oy + CELL_SIZE / 2)

    # ── routing queries ───────────────────────────────────────────────────────

    def is_blocked(self, r: int, c: int,
                   exclude: frozenset | None = None) -> bool:
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols:
            return True
        if exclude and (r, c) in exclude:
            return False
        return (r, c) in self._blocked

    def congestion_cost(self, r: int, c: int) -> int:
        """Extra traversal cost based on how many wires already use this cell."""
        return self._congestion.get((r, c), 0) * CONGESTION_PENALTY

    def mark_cell(self, r: int, c: int):
        """Record that a routed wire passes through (r, c)."""
        self._congestion[(r, c)] = self._congestion.get((r, c), 0) + 1


# ── A* search ─────────────────────────────────────────────────────────────────

_DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1))   # N S W E


def astar_route(grid: RouterGrid,
                start: QPointF, end: QPointF) -> list[QPointF] | None:
    """
    Obstacle-avoiding orthogonal A* with congestion penalty.
    Returns simplified world-space waypoints, or None if unreachable.
    """
    sr, sc = grid.world_to_cell(start.x(), start.y())
    er, ec = grid.world_to_cell(end.x(), end.y())

    if (sr, sc) == (er, ec):
        return [start, end]

    exclude = frozenset({(sr, sc), (er, ec)})

    def h(r: int, c: int) -> int:
        return abs(r - er) + abs(c - ec)

    heap = [(h(sr, sc), 0, sr, sc, -1)]
    g_scores: dict[tuple, int] = {(sr, sc, -1): 0}
    came_from: dict[tuple, tuple] = {}

    while heap:
        f, g, r, c, d = heapq.heappop(heap)
        state = (r, c, d)
        if g > g_scores.get(state, 10**9):
            continue

        if r == er and c == ec:
            cells: list[tuple[int, int]] = []
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


def simplify_path(pts: list[QPointF]) -> list[QPointF]:
    """Remove collinear intermediate points."""
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


def pin_exit_point(pin: "PinItem") -> QPointF:
    p   = pin.scene_pos()
    off = PIN_EXIT_CELLS * CELL_SIZE
    return {
        'right':  QPointF(p.x() + off, p.y()),
        'left':   QPointF(p.x() - off, p.y()),
        'bottom': QPointF(p.x(), p.y() + off),
        'top':    QPointF(p.x(), p.y() - off),
    }.get(pin.side, p)


def _mark_path(grid: RouterGrid, p_start: QPointF,
               waypoints: list[QPointF], p_end: QPointF):
    """Mark every grid cell along the full routed path as congested."""
    pts = [p_start] + waypoints + [p_end]
    for i in range(len(pts) - 1):
        r1, c1 = grid.world_to_cell(pts[i].x(), pts[i].y())
        r2, c2 = grid.world_to_cell(pts[i + 1].x(), pts[i + 1].y())
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

    def __init__(self, pin_id: int, side: str, parent_component: "ComponentItem"):
        s = self.SIZE
        super().__init__(-s / 2, -s / 2, s, s)
        self.pin_id = pin_id
        self.side = side
        self.parent_component = parent_component
        self.connections: list["ConnectionItem"] = []

        self.setPen(QPen(QColor(30, 30, 30), 1.2))
        self.setBrush(QBrush(QColor(240, 220, 40)))
        self.setZValue(3)

    def scene_pos(self) -> QPointF:
        return self.mapToScene(self.rect().center())
        # return self.mapToScene(QPointF(0, 0))

    def paint(self, painter, option, widget=None):
        super().paint(painter, option, widget)
        painter.setFont(QFont("Arial", 5))
        painter.setPen(QPen(QColor(10, 10, 10)))
        painter.drawText(self.boundingRect(), Qt.AlignmentFlag.AlignCenter,
                         str(self.pin_id))


# ── Component ─────────────────────────────────────────────────────────────────

class ComponentItem(QGraphicsItem):
    MIN_W, MAX_W = 90, 160
    MIN_H, MAX_H = 70, 130

    def __init__(self, comp_id: int, comp_type: str, color: QColor):
        super().__init__()
        self.comp_id   = comp_id
        self.comp_type = comp_type
        self.color     = color
        self.w = random.randint(self.MIN_W, self.MAX_W)
        self.h = random.randint(self.MIN_H, self.MAX_H)
        self.pins: list[PinItem] = []
        self._build_pins()
        self._init_flags()

    def _init_flags(self):
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.setZValue(1)

    def _build_pins(self):
        pid = 0

        def add(side, x, y):
            nonlocal pid
            pin = PinItem(pid, side, self)
            pin.setParentItem(self)
            pin.setPos(x, y)
            self.pins.append(pin)
            pid += 1

        w, h = self.w, self.h
        for side, count in [('top',    random.randint(1, 3)),
                             ('bottom', random.randint(1, 3)),
                             ('left',   random.randint(1, 3)),
                             ('right',  random.randint(1, 3))]:
            for i in range(count):
                f = (i + 1) / (count + 1)
                if side == 'top':      add(side, w * f, 0)
                elif side == 'bottom': add(side, w * f, h)
                elif side == 'left':   add(side, 0,     h * f)
                elif side == 'right':  add(side, w,     h * f)

    def boundingRect(self) -> QRectF:
        return QRectF(-4, -4, self.w + 8, self.h + 8)

    def paint(self, painter: QPainter, option, widget=None):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.w, self.h

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 45)))
        painter.drawRoundedRect(QRectF(5, 5, w, h), 6, 6)

        painter.setBrush(QBrush(self.color))
        border = (QPen(QColor(255, 165, 0), 2.5) if self.isSelected()
                  else QPen(QColor(30, 30, 30), 1.5))
        painter.setPen(border)
        painter.drawRoundedRect(QRectF(0, 0, w, h), 6, 6)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(self.color.darker(150)))
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

        painter.setPen(QPen(QColor(180, 180, 180)))
        painter.setFont(QFont("Arial", 6))
        painter.drawText(QRectF(2, h - 14, w - 4, 12),
                         Qt.AlignmentFlag.AlignCenter, f"{len(self.pins)} pins")

    def itemChange(self, change, value):
        # Snap to grid before position is committed
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            x = round(value.x() / SNAP_GRID) * SNAP_GRID
            y = round(value.y() / SNAP_GRID) * SNAP_GRID
            return QPointF(x, y)

        # Stretch connections live while dragging; signal scene to reroute on release
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for pin in self.pins:
                for conn in pin.connections:
                    conn.update_path()          # stretch to new pin pos
            scene = self.scene()
            if scene is not None:
                scene._needs_reroute = True     # flag for post-drag reroute

        return super().itemChange(change, value)


# ── RFIC Component ────────────────────────────────────────────────────────────

class RFICItem(ComponentItem):
    PINS_LR = 10
    PINS_TB = 3

    def __init__(self, comp_id: int):
        QGraphicsItem.__init__(self)
        self.comp_id   = comp_id
        self.comp_type = "RFIC-2450"
        self.color     = QColor(18, 130, 95)
        self.w = 140
        self.h = 260
        self.pins: list[PinItem] = []
        self._build_pins()
        self._init_flags()

    def _build_pins(self):
        pid = 0

        def add(side, x, y):
            nonlocal pid
            pin = PinItem(pid, side, self)
            pin.setParentItem(self)
            pin.setPos(x, y)
            self.pins.append(pin)
            pid += 1

        w, h = self.w, self.h
        for i in range(self.PINS_LR):
            f = (i + 1) / (self.PINS_LR + 1)
            add('left',  0, h * f)
            add('right', w, h * f)
        for i in range(self.PINS_TB):
            f = (i + 1) / (self.PINS_TB + 1)
            add('top',    w * f, 0)
            add('bottom', w * f, h)

    def paint(self, painter: QPainter, option, widget=None):
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
        painter.drawText(QRectF(3, 3, w - 6, 20),
                         Qt.AlignmentFlag.AlignCenter, self.comp_type)

        painter.setPen(QPen(QColor(220, 255, 220)))
        painter.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
        painter.drawText(QRectF(0, 28, w, h - 50),
                         Qt.AlignmentFlag.AlignCenter, f"U{self.comp_id}")

        painter.setFont(QFont("Arial", 5))
        for pin in self.pins:
            px, py = pin.pos().x(), pin.pos().y()
            if pin.side == 'left':
                painter.setPen(QPen(QColor(200, 255, 200)))
                painter.drawText(QRectF(px + 6, py - 5, 30, 10),
                                 Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                                 f"P{pin.pin_id}")
            elif pin.side == 'right':
                painter.setPen(QPen(QColor(200, 255, 200)))
                painter.drawText(QRectF(px - 36, py - 5, 30, 10),
                                 Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                                 f"P{pin.pin_id}")

        painter.setPen(QPen(QColor(160, 230, 160)))
        painter.setFont(QFont("Arial", 6))
        painter.drawText(QRectF(2, h - 18, w - 4, 14),
                         Qt.AlignmentFlag.AlignCenter, f"{len(self.pins)} pins")


# ── Connection ────────────────────────────────────────────────────────────────

class ConnectionItem(QGraphicsPathItem):
    def __init__(self, pin1: PinItem, pin2: PinItem):
        super().__init__()
        self.pin1 = pin1
        self.pin2 = pin2
        self._waypoints: list[QPointF] | None = None

        pen = QPen(WIRE_COLOR, 1.5)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.MiterJoin)   # sharp corners for orthogonal routes
        self.setPen(pen)
        self.setZValue(0)
        self._draw_bezier()

    def set_routed_path(self, waypoints: list[QPointF]):
        self._waypoints = waypoints
        self._draw_routed()

    def update_path(self):
        """Called during drag: stretch existing waypoints to current pin positions."""
        if self._waypoints:
            self._draw_routed()
        else:
            self._draw_bezier()

    def _draw_routed(self):
        p1 = self.pin1.scene_pos()
        p2 = self.pin2.scene_pos()
        path = QPainterPath(p1)
        for pt in self._waypoints:
            path.lineTo(pt)
        path.lineTo(p2)
        self.setPath(path)

    def _draw_bezier(self):
        p1 = self.pin1.scene_pos()
        p2 = self.pin2.scene_pos()
        dist = max(abs(p2.x()-p1.x()), abs(p2.y()-p1.y())) * 0.45 + 40
        offsets = {'right': (dist, 0), 'left': (-dist, 0),
                   'bottom': (0, dist), 'top': (0, -dist)}
        ox1, oy1 = offsets.get(self.pin1.side, (0, 0))
        ox2, oy2 = offsets.get(self.pin2.side, (0, 0))
        path = QPainterPath(p1)
        path.cubicTo(QPointF(p1.x()+ox1, p1.y()+oy1),
                     QPointF(p2.x()+ox2, p2.y()+oy2), p2)
        self.setPath(path)


# ── Scene ─────────────────────────────────────────────────────────────────────

class CADScene(QGraphicsScene):
    NUM_COMPONENTS = 20
    NUM_CONNECTIONS = 35

    def __init__(self):
        super().__init__()
        self.setSceneRect(-200, -200, 2400, 1600)
        self.components:  list[ComponentItem] = []
        self.connections: list[ConnectionItem] = []
        self._needs_reroute = False
        self._populate()
        self.route_all()

    # ── population ────────────────────────────────────────────────────────────

    def _populate(self):
        types = COMPONENT_TYPES[:]
        random.shuffle(types)
        cols = 5
        for i in range(self.NUM_COMPONENTS):
            col, row = i % cols, i // cols
            x = round((col * 280 + random.uniform(-40, 40)) / SNAP_GRID) * SNAP_GRID
            y = round((row * 240 + random.uniform(-30, 30)) / SNAP_GRID) * SNAP_GRID
            comp = ComponentItem(i + 1, types[i], random.choice(COMPONENT_COLORS))
            comp.setPos(x, y)
            self.addItem(comp)
            self.components.append(comp)

        rfic = RFICItem(self.NUM_COMPONENTS + 1)
        rfic.setPos(1720, 200)
        self.addItem(rfic)
        self.components.append(rfic)

        all_pins: list[PinItem] = [p for c in self.components for p in c.pins]
        used: set[frozenset] = set()
        created = 0
        attempts = 0
        target = self.NUM_CONNECTIONS + 8
        while created < target and attempts < 3000:
            attempts += 1
            pa, pb = random.choice(all_pins), random.choice(all_pins)
            if pa is pb or pa.parent_component is pb.parent_component:
                continue
            key = frozenset({id(pa), id(pb)})
            if key in used:
                continue
            used.add(key)
            conn = ConnectionItem(pa, pb)
            self.addItem(conn)
            pa.connections.append(conn)
            pb.connections.append(conn)
            self.connections.append(conn)
            created += 1

    # ── A* routing ────────────────────────────────────────────────────────────

    def route_all(self) -> tuple[int, int]:
        """
        Build a fresh routing grid (with congestion) and A*-route all connections.
        Shorter connections are routed first so they get cleaner paths.
        """
        grid = RouterGrid(self.components)
        self._needs_reroute = False

        # Sort by Manhattan distance — shorter nets get priority
        def net_length(conn: ConnectionItem) -> float:
            p1, p2 = conn.pin1.scene_pos(), conn.pin2.scene_pos()
            return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y())

        sorted_conns = sorted(self.connections, key=net_length)

        routed = fallback = 0
        for conn in sorted_conns:
            e1 = pin_exit_point(conn.pin1)
            e2 = pin_exit_point(conn.pin2)
            path = astar_route(grid, e1, e2)
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

    def drawBackground(self, painter: QPainter, rect):
        painter.fillRect(rect, QColor(255, 255, 255))

        # Coarse grids first
        for step, color, width in [
            (GRID_MAJOR, QColor(160, 160, 175), 1.2),
            (GRID_MINOR, QColor(195, 195, 205), 0.8),
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

    def __init__(self, scene: CADScene):
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
            # Re-route ALL connections after any component drag
            if event.button() == Qt.MouseButton.LeftButton:
                s = self.scene()
                if getattr(s, '_needs_reroute', False):
                    s.route_all()


# ── Main Window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAD Schematic Viewer — A* Routing")
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
            "  Scroll=Zoom  |  Middle-drag=Pan  |  Click=Select  "
            "|  Drag=Move (snaps grid, auto-reroutes on release)"))

    def _refresh_status(self, routed=None, fallback=None):
        n_comp = len(self.cad_scene.components)
        n_conn = len(self.cad_scene.connections)
        n_pins = sum(len(c.pins) for c in self.cad_scene.components)
        extra  = (f"   |   A* routed: {routed}   Bezier fallback: {fallback}"
                  if routed is not None else "")
        self._status.showMessage(
            f"Components: {n_comp}   Pins: {n_pins}   Connections: {n_conn}{extra}")

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
