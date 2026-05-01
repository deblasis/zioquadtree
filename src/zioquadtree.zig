//! Quadtree for 2D spatial partitioning.
//!
//! Efficient broad-phase collision detection, frustum culling,
//! and spatial queries. Insert/query/remove AABB items.

const std = @import("std");

/// Default max depth to prevent infinite recursion.
pub const DEFAULT_MAX_DEPTH = 6;

/// Axis-aligned bounding box for quadtree regions.
pub const Bounds = struct {
    x: f32,
    y: f32,
    w: f32,
    h: f32,

    pub fn contains(b: Bounds, px: f32, py: f32) bool {
        return px >= b.x and px < b.x + b.w and py >= b.y and py < b.y + b.h;
    }

    pub fn intersects(a: Bounds, b: Bounds) bool {
        return a.x < b.x + b.w and a.x + a.w > b.x and
            a.y < b.y + b.h and a.y + a.h > b.y;
    }

    pub fn centerX(b: Bounds) f32 {
        return b.x + b.w / 2;
    }
    pub fn centerY(b: Bounds) f32 {
        return b.y + b.h / 2;
    }
};

/// An item in the quadtree.
pub const Item = struct {
    bounds: Bounds,
    data: u32,
};

const MAX_NODE_ITEMS: usize = 8;

/// A quadtree node. Uses fixed-capacity arrays.
pub const QuadNode = struct {
    bounds: Bounds,
    items: [MAX_NODE_ITEMS]Item,
    item_count: usize,
    children: ?*[4]*QuadNode,
    max_depth: u32,
    depth: u32,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, bounds: Bounds, max_depth: u32, depth: u32) !*QuadNode {
        const node = try allocator.create(QuadNode);
        node.* = .{
            .bounds = bounds,
            .items = undefined,
            .item_count = 0,
            .children = null,
            .max_depth = max_depth,
            .depth = depth,
            .allocator = allocator,
        };
        return node;
    }

    pub fn deinit(self: *QuadNode) void {
        if (self.children) |ch| {
            for (ch) |child| {
                child.deinit();
            }
            self.allocator.destroy(ch);
        }
        self.allocator.destroy(self);
    }

    /// Insert an item. Returns false if item is outside bounds.
    pub fn insert(self: *QuadNode, bounds: Bounds, data: u32) !bool {
        if (!self.bounds.intersects(bounds)) return false;

        if (self.children) |ch| {
            for (ch) |child| {
                _ = try child.insert(bounds, data);
            }
            return true;
        }

        if (self.item_count < MAX_NODE_ITEMS) {
            self.items[self.item_count] = .{ .bounds = bounds, .data = data };
            self.item_count += 1;
        }

        if (self.item_count >= MAX_NODE_ITEMS and self.depth < self.max_depth) {
            try self.split();
        }
        return true;
    }

    /// Query all items overlapping the given bounds. Caller provides result buffer.
    pub fn query(self: *const QuadNode, region: Bounds, result: []Item, result_count: *usize) void {
        if (!self.bounds.intersects(region)) return;

        for (self.items[0..self.item_count]) |item| {
            if (item.bounds.intersects(region)) {
                if (result_count.* < result.len) {
                    result[result_count.*] = item;
                    result_count.* += 1;
                }
            }
        }

        if (self.children) |ch| {
            for (ch) |child| {
                child.query(region, result, result_count);
            }
        }
    }

    /// Count all items in the tree.
    pub fn count(self: *const QuadNode) usize {
        var n = self.item_count;
        if (self.children) |ch| {
            for (ch) |child| {
                n += child.count();
            }
        }
        return n;
    }

    /// Clear all items and children.
    pub fn clear(self: *QuadNode) void {
        self.item_count = 0;
        if (self.children) |ch| {
            for (ch) |child| {
                child.deinit();
            }
            self.allocator.destroy(ch);
            self.children = null;
        }
    }

    fn split(self: *QuadNode) !void {
        if (self.children != null) return;

        const hw = self.bounds.w / 2;
        const hh = self.bounds.h / 2;
        const cx = self.bounds.centerX();
        const cy = self.bounds.centerY();
        const next_depth = self.depth + 1;

        const quadrants = [4]Bounds{
            .{ .x = self.bounds.x, .y = self.bounds.y, .w = hw, .h = hh },
            .{ .x = cx, .y = self.bounds.y, .w = hw, .h = hh },
            .{ .x = self.bounds.x, .y = cy, .w = hw, .h = hh },
            .{ .x = cx, .y = cy, .w = hw, .h = hh },
        };

        const children = try self.allocator.create([4]*QuadNode);
        for (children, 0..) |_, i| {
            children[i] = try QuadNode.init(self.allocator, quadrants[i], self.max_depth, next_depth);
        }
        self.children = children;

        const old_count = self.item_count;
        self.item_count = 0;
        for (self.items[0..old_count]) |item| {
            for (children) |child| {
                if (child.bounds.intersects(item.bounds)) {
                    if (child.item_count < MAX_NODE_ITEMS) {
                        child.items[child.item_count] = item;
                        child.item_count += 1;
                    }
                }
            }
        }
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "QuadNode insert and query" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2);

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(.{ .x = 0, .y = 0, .w = 20, .h = 20 }, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 1), result_count);
    try std.testing.expectEqual(@as(u32, 1), result[0].data);
}

test "QuadNode insert outside bounds returns false" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    const ok = try node.insert(.{ .x = 200, .y = 200, .w = 5, .h = 5 }, 1);
    try std.testing.expect(!ok);
}

test "QuadNode splits on overflow" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    for (0..10) |i| {
        _ = try node.insert(.{ .x = @floatFromInt(i * 5), .y = @floatFromInt(i * 5), .w = 2, .h = 2 }, @intCast(i));
    }

    try std.testing.expect(node.children != null);
}

test "QuadNode query all" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    for (0..8) |i| {
        _ = try node.insert(.{ .x = @floatFromInt(i * 10), .y = 0, .w = 2, .h = 2 }, @intCast(i));
    }

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(bounds, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 8), result_count);
}

test "QuadNode count" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    for (0..8) |i| {
        _ = try node.insert(.{ .x = @floatFromInt(i * 10), .y = 0, .w = 2, .h = 2 }, @intCast(i));
    }
    try std.testing.expectEqual(@as(usize, 8), node.count());
}

test "QuadNode clear" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    for (0..10) |i| {
        _ = try node.insert(.{ .x = @floatFromInt(i * 10), .y = 0, .w = 2, .h = 2 }, @intCast(i));
    }
    node.clear();
    try std.testing.expectEqual(@as(usize, 0), node.count());
    try std.testing.expect(node.children == null);
}

test "Bounds contains" {
    const b = Bounds{ .x = 10, .y = 10, .w = 20, .h = 20 };
    try std.testing.expect(b.contains(15, 15));
    try std.testing.expect(!b.contains(5, 5));
}

test "Bounds intersects" {
    const a = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = Bounds{ .x = 5, .y = 5, .w = 10, .h = 10 };
    const c = Bounds{ .x = 20, .y = 20, .w = 5, .h = 5 };
    try std.testing.expect(a.intersects(b));
    try std.testing.expect(!a.intersects(c));
}

test "Bounds self-intersects" {
    const a = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    try std.testing.expect(a.intersects(a));
}

test "Bounds contains edge" {
    const b = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    try std.testing.expect(b.contains(0, 0)); // top-left corner
    try std.testing.expect(!b.contains(10, 10)); // outside (exclusive)
}

test "QuadNode max depth prevents split" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    // max_depth = 0 means no splitting ever
    var node = try QuadNode.init(std.testing.allocator, bounds, 0, 0);
    defer node.deinit();

    for (0..20) |i| {
        _ = try node.insert(.{ .x = @floatFromInt(i * 3), .y = @floatFromInt(i * 3), .w = 2, .h = 2 }, @intCast(i));
    }
    try std.testing.expect(node.children == null);
}

test "QuadNode partial overlap query" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 45, .y = 45, .w = 10, .h = 10 }, 1); // straddles center

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(.{ .x = 48, .y = 0, .w = 10, .h = 100 }, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 1), result_count);
}

test "QuadNode nested splits" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 4, 0);
    defer node.deinit();

    // Insert enough items to trigger multiple splits
    for (0..30) |i| {
        _ = try node.insert(.{ .x = @as(f32, @floatFromInt(i)) * 3.1, .y = @as(f32, @floatFromInt(i)) * 3.1, .w = 1, .h = 1 }, @intCast(i));
    }
    try std.testing.expect(node.children != null);
    try std.testing.expect(node.count() >= 25); // some items may be on boundaries
}

test "Bounds zero size" {
    const b = Bounds{ .x = 5, .y = 5, .w = 0, .h = 0 };
    // Zero-width bounds don't contain anything since x < x+w fails
    try std.testing.expect(!b.contains(5, 5));
    // Zero-size bounds don't intersect with anything that doesn't overlap exactly
    const other = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    // b.x(5) < other.maxX(10) and b.maxX(5) > other.x(0) => true
    try std.testing.expect(other.intersects(b));
}

test "QuadNode empty query" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(bounds, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 0), result_count);
}

test "QuadNode query outside region" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };

    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(.{ .x = 50, .y = 50, .w = 10, .h = 10 }, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 0), result_count);
}

test "Bounds center calculation" {
    const b = Bounds{ .x = 10, .y = 20, .w = 30, .h = 40 };
    try std.testing.expectEqual(@as(f32, 25), b.centerX());
    try std.testing.expectEqual(@as(f32, 40), b.centerY());
}

test "QuadNode insert at corner" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    // Item at exact corner
    _ = try node.insert(.{ .x = 0, .y = 0, .w = 1, .h = 1 }, 1);
    try std.testing.expectEqual(@as(usize, 1), node.count());
}

test "QuadNode overlapping items" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    // Two items at same position
    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);
    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 2);

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(.{ .x = 5, .y = 5, .w = 20, .h = 20 }, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 2), result_count);
}

test "Bounds contains on edge" {
    const b = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    // Left edge is included
    try std.testing.expect(b.contains(0, 5));
    // Right edge is excluded
    try std.testing.expect(!b.contains(10, 5));
}

test "QuadNode single item query" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 99);

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(.{ .x = 0, .y = 0, .w = 100, .h = 100 }, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 1), result_count);
    try std.testing.expectEqual(@as(u32, 99), result[0].data);
}

test "Bounds non-intersecting adjacent" {
    const a = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = Bounds{ .x = 10, .y = 10, .w = 10, .h = 10 };
    // a.maxX == b.x and a.maxY == b.y, but strict inequality means no overlap
    try std.testing.expect(!a.intersects(b));
}

test "QuadNode insert many clustered items" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    // All items clustered in one corner
    for (0..8) |i| {
        _ = try node.insert(.{ .x = 1, .y = @floatFromInt(i), .w = 2, .h = 1 }, @intCast(i));
    }
    try std.testing.expectEqual(@as(usize, 8), node.count());
}

test "QuadNode clear and reinsert" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2);
    try std.testing.expectEqual(@as(usize, 2), node.count());

    node.clear();
    try std.testing.expectEqual(@as(usize, 0), node.count());

    _ = try node.insert(.{ .x = 80, .y = 80, .w = 5, .h = 5 }, 3);
    try std.testing.expectEqual(@as(usize, 1), node.count());
}

test "Bounds large values" {
    const b = Bounds{ .x = -1000, .y = -1000, .w = 2000, .h = 2000 };
    try std.testing.expect(b.contains(0, 0));
    try std.testing.expect(b.contains(-500, 500));
    try std.testing.expect(!b.contains(2000, 0));
}

test "QuadNode items at boundaries" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();

    // Item exactly at center
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 1, .h = 1 }, 1);
    // Item at edge
    _ = try node.insert(.{ .x = 0, .y = 0, .w = 1, .h = 1 }, 2);
    try std.testing.expectEqual(@as(usize, 2), node.count());
}

test "Bounds intersects with itself" {
    const b = Bounds{ .x = 5, .y = 5, .w = 10, .h = 10 };
    try std.testing.expect(b.intersects(b));
}

test "QuadNode query with small region" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();
    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2);

    var result: [64]Item = undefined;
    var result_count: usize = 0;
    node.query(.{ .x = 9, .y = 9, .w = 7, .h = 7 }, &result, &result_count);
    try std.testing.expectEqual(@as(usize, 1), result_count);
}

test "Bounds contains center point" {
    const b = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    try std.testing.expect(b.contains(5, 5));
}

test "QuadNode insert and count matches" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 200, .h = 200 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();
    for (0..8) |i| {
        _ = try node.insert(.{ .x = @as(f32, @floatFromInt(i)) * 23.7, .y = @as(f32, @floatFromInt(i)) * 23.7, .w = 5, .h = 5 }, @intCast(i));
    }
    try std.testing.expect(node.count() >= 8);
}

test "QuadNode query finds items in overlapping regions" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 4, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 25, .y = 25, .w = 10, .h = 10 }, 1);
    _ = try node.insert(.{ .x = 75, .y = 75, .w = 10, .h = 10 }, 2);
    _ = try node.insert(.{ .x = 25, .y = 75, .w = 10, .h = 10 }, 3);

    // Query overlapping center should find all 3
    var result: [64]Item = undefined;
    var count: usize = 0;
    node.query(.{ .x = 0, .y = 0, .w = 100, .h = 100 }, &result, &count);
    try std.testing.expect(count >= 3);
}

test "Bounds non-overlapping regions" {
    const a = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = Bounds{ .x = 20, .y = 20, .w = 10, .h = 10 };
    try std.testing.expect(!a.intersects(b));
}

test "QuadNode count never decreases without clear" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 4, 0);
    defer node.deinit();

    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);
    const c1 = node.count();
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2);
    const c2 = node.count();
    try std.testing.expect(c2 >= c1);
}

test "Bounds intersects is reflexive and symmetric" {
    const a = Bounds{ .x = 0, .y = 0, .w = 10, .h = 10 };
    const b = Bounds{ .x = 5, .y = 5, .w = 10, .h = 10 };
    try std.testing.expect(a.intersects(a)); // reflexive
    try std.testing.expectEqual(a.intersects(b), b.intersects(a)); // symmetric
}

test "QuadNode query empty tree returns zero" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 4, 0);
    defer node.deinit();

    var result: [64]Item = undefined;
    var count: usize = 0;
    node.query(.{ .x = 0, .y = 0, .w = 100, .h = 100 }, &result, &count);
    try std.testing.expectEqual(@as(usize, 0), count);
}

test "QuadNode clear on empty tree" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 4, 0);
    defer node.deinit();
    node.clear(); // should not crash
    try std.testing.expectEqual(@as(usize, 0), node.count());
}

test "QuadNode insert at same position" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 4, 0);
    defer node.deinit();
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 1);
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2);
    try std.testing.expectEqual(@as(usize, 2), node.count());
}

test "example: 3 items in tree" {
    const bounds = Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var node = try QuadNode.init(std.testing.allocator, bounds, 6, 0);
    defer node.deinit();
    _ = try node.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1);
    _ = try node.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2);
    _ = try node.insert(.{ .x = 80, .y = 80, .w = 5, .h = 5 }, 3);
    try std.testing.expectEqual(@as(usize, 3), node.count());
}
