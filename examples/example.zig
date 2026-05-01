const std = @import("std");
const qtree = @import("zioquadtree");

pub fn main() !void {
    const bounds = qtree.Bounds{ .x = 0, .y = 0, .w = 1000, .h = 1000 };
    var tree = qtree.QuadNode.init(std.heap.page_allocator, bounds, 8, 0) catch @panic("OOM");
    defer tree.deinit();

    // Insert items (enemies, bullets, etc.)
    _ = tree.insert(.{ .x = 100, .y = 200, .w = 32, .h = 32 }, 1) catch {};
    _ = tree.insert(.{ .x = 500, .y = 300, .w = 16, .h = 16 }, 2) catch {};
    _ = tree.insert(.{ .x = 105, .y = 195, .w = 32, .h = 32 }, 3) catch {};
    std.debug.print("Total items: {}\n", .{tree.count()});

    // Query for nearby items (camera view around the two enemies)
    var result: [64]qtree.Item = undefined;
    var count: usize = 0;
    tree.query(.{ .x = 90, .y = 190, .w = 60, .h = 60 }, &result, &count);
    std.debug.print("Query near (100,200): {} items\n", .{count});
    for (result[0..count]) |item| {
        std.debug.print("  item data={} at ({d:.0},{d:.0})\n", .{ item.data, item.bounds.x, item.bounds.y });
    }

    // Clear and reuse
    tree.clear();
    std.debug.print("After clear: {} items\n", .{tree.count()});
}
