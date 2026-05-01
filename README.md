# zioquadtree

> Quadtree for 2D spatial partitioning in Zig. Broad-phase collision, spatial queries.

Part of the [zio-zig](https://github.com/deblasis/zio-zig) ecosystem.

## Quick start

```zig
const qtree = @import("zioquadtree");

// Create a quadtree covering the game world
const bounds = qtree.Bounds{ .x = 0, .y = 0, .w = 1000, .h = 1000 };
var tree = try qtree.QuadNode.init(allocator, bounds, 8, 0);
//                                       alloc  bounds  capacity  depth
defer tree.deinit();

// Insert items (enemies, bullets, etc.)
try tree.insert(.{ .x = 100, .y = 200, .w = 32, .h = 32 }, enemy_id_1);
try tree.insert(.{ .x = 500, .y = 300, .w = 16, .h = 16 }, bullet_id_1);
try tree.insert(.{ .x = 105, .y = 195, .w = 32, .h = 32 }, enemy_id_2);

// Query for nearby items
var result: [64]qtree.Item = undefined;
var count: usize = 0;
tree.query(.{ .x = 90, .y = 190, .w = 60, .h = 60 }, &result, &count);
// count = 2 (both enemies, but not the bullet)

// Clear and reuse
tree.clear();
```

```bash
zig build test          # Run 37 tests
zig build run-example   # Run example
```

## Example output

```
$ zig build run-example
Total items: 3
```

## API

### Types

| Type | Fields | Description |
|------|--------|-------------|
| `Bounds` | `x, y, w, h` | Axis-aligned rectangle |
| `Item` | `bounds: Bounds`, `id: usize` | Insertable item |

### Bounds methods
- `contains(px, py)` — point containment
- `intersects(b)` — overlap test

### QuadNode

| Method | Description |
|--------|-------------|
| `init(allocator, bounds, capacity, depth)` | Create node |
| `deinit()` | Free memory |
| `insert(item_bounds, id)` | Insert item (may subdivide) |
| `query(region, results, count)` | Find items overlapping region |
| `count()` | Total items in tree |
| `clear()` | Remove all items (keep structure) |

## License

MIT. Copyright (c) 2026 Alessandro De Blasis.
