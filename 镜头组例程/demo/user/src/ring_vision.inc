/* Included by img_process.c so existing IDE project file lists stay valid.
 * Moore eight-neighbour tracing, in canonical right-ring coordinates.
 * A: lower opening corner. B: inward x extremum. C: downward y extremum.
 * No screenshot direction numbers or gyro accumulator units are reused.
 */
#define RING_TRACE_MAX 1024
#define RING_POINT_SPAN 4
typedef struct { int16_t x, y; } ring_trace_point_t;
static ring_trace_point_t ring_trace[RING_TRACE_MAX];

static int ring_white(const uint8_t *im, int x, int y, int rows, int cols, int side)
{
    if (x < 0 || x >= cols || y < 0 || y >= rows) return 0;
    return im[y * cols + (side ? x : cols - 1 - x)] > 127;
}

static int ring_trace_edge(const uint8_t *im, int sx, int sy, int rows, int cols, int side)
{
    // Counterclockwise: start with the black neighbour to the right.
    static const int dx[8] = {1,1,0,-1,-1,-1,0,1};
    static const int dy[8] = {0,-1,-1,-1,0,1,1,1};
    int x = sx, y = sy, back = 0, count = 0;
    while (count < RING_TRACE_MAX) {
        ring_trace[count].x = (int16_t)x;
        ring_trace[count++].y = (int16_t)y;
        if (y == 0 || (count > 8 && y >= rows - 1)) break;
        int found = -1;
        for (int k = 0; k < 8; ++k) {
            int d = (back + k) & 7;
            if (ring_white(im, x + dx[d], y + dy[d], rows, cols, side)) {
                found = d; break;
            }
        }
        if (found < 0) break;
        int prev = (found + 7) & 7;
        int bx = x + dx[prev], by = y + dy[prev];
        x += dx[found]; y += dy[found];
        for (int d = 0; d < 8; ++d)
            if (dx[d] == bx - x && dy[d] == by - y) { back = d; break; }
        if (x == sx && y == sy) break;
    }
    return count;
}

static ring_point_t ring_point(int i, int cols, int side)
{
    ring_point_t p;
    p.x = (uint8_t)(side ? ring_trace[i].x : cols - 1 - ring_trace[i].x);
    p.y = (uint8_t)ring_trace[i].y;
    p.valid = 1;
    return p;
}

static void ring_find_side(ring_side_t *out, const uint8_t *im,
    const uint8_t *edge, int rows, int cols, int side)
{
    int start = rows / 4, end = rows - 2;
    int up = 0, down = 0, valid = 0, missing = 0, previous = -1;
    int sx = -1, sy = -1;
    memset(out, 0, sizeof *out);
    // When A leaves the bottom of the image, start on the image-border
    // continuation of the white road; this still lets us trace B and C.
    if (ring_white(im, cols-1, end, rows, cols, side)) { sx = cols-1; sy = end; }
    for (int y = end; y >= start; --y) {
        int x = side ? edge[y] : cols - 1 - edge[y];
        if (x <= 0 || x >= cols - 1) { ++missing; previous = -1; continue; }
        ++valid;
        if (previous >= 0) {
            int delta = x - previous;
            if (delta > 1) up += delta;
            if (delta < -1) down -= delta;
        }
        previous = x;
        if (sx < 0 && y >= rows * 3 / 4 && ring_white(im, x - 1, y, rows, cols, side)) {
            sx = x - 1; sy = y;
        }
    }
    out->lost = missing >= rows / 20;
    out->monotonic = valid >= (end - start) * 9 / 10 && !out->lost &&
        (up < cols / 20 || down < cols / 20);
    if (sx < 0) return;
    int n = ring_trace_edge(im, sx, sy, rows, cols, side);
    if (n == RING_TRACE_MAX) return; // incomplete contour cannot supply corners
    int ai = -1, bi = -1;
    int allow_a = 1;
    for (int i = RING_POINT_SPAN; i + RING_POINT_SPAN < n; ++i) {
        int x = ring_trace[i].x, y = ring_trace[i].y;
        if (x < cols / 10 || x >= cols - 3) continue;
        // A: upward edge turns outward along the lower lip of the opening.
        if (allow_a && ai < 0 && y >= rows / 3 &&
            ring_trace[i - 4].y - y >= 2 &&
            ring_trace[i + 4].x - x >= 3 &&
            ring_trace[i + 4].y >= y - 1) {
            ai = i; out->a = ring_point(i, cols, side);
        }
        int xmin = 1, ymax = 1;
        for (int k = 1; k <= 4; ++k) {
            if (x > ring_trace[i-k].x || x > ring_trace[i+k].x) xmin = 0;
            if (y < ring_trace[i-k].y || y < ring_trace[i+k].y) ymax = 0;
        }
        // Strict prominence prevents a straight/flat edge from becoming B/C.
        if (bi < 0 && xmin && ring_trace[i-4].x - x >= 2 &&
            ring_trace[i+4].x - x >= 2 &&
            (ai < 0 || (i > ai + 8 && y < out->a.y - rows / 20))) {
            bi = i; out->b = ring_point(i, cols, side);
        }
        if (bi >= 0 && i > bi + 8 && i + 8 < n && ymax &&
            y - ring_trace[i-8].y >= 2 && y - ring_trace[i+8].y >= 2 &&
            x < ring_trace[bi].x && y < ring_trace[bi].y) {
            out->c = ring_point(i, cols, side); break;
        }
    }
}

track_element_t track_visual_classify(ring_vision_t *vision, const uint8_t *im,
    const uint8_t *left, const uint8_t *right, uint8_t rows, uint8_t cols)
{
    memset(vision, 0, sizeof *vision);
    vision->enabled = 1;
    if (!im || !left || !right || rows < 40 || cols < 40) return TRACK_UNKNOWN;
    ring_find_side(&vision->side[0], im, left, rows, cols, 0);
    ring_find_side(&vision->side[1], im, right, rows, cols, 1);
    // 识别只看A/B入口几何和另一侧连续边线；C点易抖动，只做导航提示，不作识别门槛。
    for (int side = 0; side < 2; ++side) {
        ring_side_t *s = &vision->side[side];
        // A/B已按轮廓跟踪方向归一化；左右侧都要求B位于A的内侧（x更小）。
        int inward_dx = (int)s->a.x - s->b.x;
        if (s->lost && !s->monotonic && vision->side[1-side].monotonic &&
            s->a.valid && s->b.valid &&
            s->a.y > s->b.y &&
            inward_dx >= cols / 40 &&
            s->a.y - s->b.y >= rows / 10)
            return side ? TRACK_ROUND_RIGHT : TRACK_ROUND_LEFT;
    }
    // 只复用十字判据；旧的开口序列识别不参与生产环岛判断。
    track_element_t old = track_cross_classify(left, right, rows, cols);
    return old == TRACK_CROSS || old == TRACK_NONE ? old : TRACK_UNKNOWN;
}\n