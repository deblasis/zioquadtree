const std = @import("std");
const zq = @import("zioquadtree");

pub fn main() !void {
    const bounds = zq.Bounds{ .x = 0, .y = 0, .w = 100, .h = 100 };
    var tree = zq.QuadNode.init(std.page_allocator, bounds, 6, 0) catch @panic("OOM");
    defer tree.deinit();

    _ = tree.insert(.{ .x = 10, .y = 10, .w = 5, .h = 5 }, 1) catch {};
    _ = tree.insert(.{ .x = 50, .y = 50, .w = 5, .h = 5 }, 2) catch {};
    _ = tree.insert(.{ .x = 80, .y = 80, .w = 5, .h = 5 }, 3) catch {};

    std.debug.print("Total items: {}\n", .{tree.count()});
}
