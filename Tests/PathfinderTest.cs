using System.IO;
using NUnit.Framework;
using TowerDefense;
using Unity.Collections;
using Unity.Mathematics;

public class PathfinderTest
{
    [Test]
    public void TargetOnMapBorder_DoesNotReturnAPath()
    {
        ParseMap(new[] {" "}, out var weights, out var dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        var r = pathfinder.FindPath(new int2(1, 1), new int2(0, 1));
        Assert.IsFalse(r);
        r = pathfinder.GetPath(default, new int2(0, 1));
        Assert.IsFalse(r);
    }

    [Test]
    public void StartingLocationOnMapBorder_DoesNotReturnAPath()
    {
        ParseMap(new[] {" "}, out var weights, out var dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        var r = pathfinder.FindPath(new int2(0, 1), new int2(1, 1));
        Assert.IsFalse(r);
        r = pathfinder.GetPath(default, new int2(1, 1));
        Assert.IsFalse(r);
    }

    [Test]
    public void ShortCorridor_ReturnsRightPath([Values("000", "010")] string map)
    {
        ParseMap(new[] {map}, out var weights, out var dim);
        Assert.AreEqual(new int2(5, 3), dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        {
            var r = pathfinder.FindPath(new int2(1, 1), new int2(3, 1));
            Assert.IsTrue(r);
        }

        {
            var path = new NativeList<int2>(Allocator.Temp);
            var r = pathfinder.GetPath(path, new int2(3, 1));
            Assert.IsTrue(r);
            Assert.AreEqual(2, path.Length);
            Assert.AreEqual(new int2(3, 1), path[0]);
            Assert.AreEqual(new int2(1, 1), path[1]);
        }
    }

    private static readonly string[] OpenField = {
        "     ",
        "     ",
        "     ",
        "     ",
        "     ",
    };

    [Test]
    public void OpenField_WithStraightPath_ReturnsRightPath()
    {
        ParseMap(OpenField, out var weights, out var dim);
        Assert.AreEqual(new int2(7, 7), dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        {
            var r = pathfinder.FindPath(new int2(1, 1), new int2(1, 5));
            Assert.IsTrue(r);
        }

        {
            var path = new NativeList<int2>(Allocator.Temp);
            var r = pathfinder.GetPath(path, new int2(1, 5));
            Assert.IsTrue(r);
            Assert.AreEqual(2, path.Length);
            Assert.AreEqual(new int2(1, 5), path[0]);
            Assert.AreEqual(new int2(1, 1), path[1]);
        }
    }

    [Test]
    public void OpenField_OppositeBorders_ReturnsRightPath()
    {
        ParseMap(OpenField, out var weights, out var dim);
        Assert.AreEqual(new int2(7, 7), dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        {
            var r = pathfinder.FindPath(new int2(1, 1), new int2(5, 5));
            Assert.IsTrue(r);
        }

        {
            var path = new NativeList<int2>(Allocator.Temp);
            var r = pathfinder.GetPath(path, new int2(5, 5));
            Assert.IsTrue(r);
            Assert.AreEqual(2, path.Length);
            Assert.AreEqual(new int2(5, 5), path[0]);
            Assert.AreEqual(new int2(1, 1), path[1]);
        }
    }

    private static readonly string[] SmallWalls =
    {
        "X ",
        "  ",
    };

    [Test]
    public void SmallWalls_WithOppositeCorners_ReturnsRightPath()
    {
        ParseMap(SmallWalls, out var weights, out var dim);
        Assert.AreEqual(new int2(4, 4), dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        {
            var r = pathfinder.FindPath(new int2(1, 2), new int2(2, 1));
            Assert.IsTrue(r);
        }

        {
            var path = new NativeList<int2>(Allocator.Temp);
            var r = pathfinder.GetPath(path, new int2(2, 1));
            Assert.IsTrue(r);
            UnityEngine.Debug.Log(string.Join(", ", path.ToArray()));
            Assert.AreEqual(3, path.Length);
            Assert.AreEqual(new int2(2, 1), path[0]);
            Assert.AreEqual(new int2(2, 2), path[1]);
            Assert.AreEqual(new int2(1, 2), path[2]);
        }
    }

    private static readonly string[] Walls = {
        " X   ",
        " X X ",
        " X X ",
        " X X ",
        "   X ",
    };

    [Test]
    public void Walls_WithOppositeCorners_ReturnsRightPath()
    {
        ParseMap(Walls, out var weights, out var dim);
        Assert.AreEqual(new int2(7, 7), dim);
        var pathfinder = new Pathfinder(dim, weights, Allocator.Temp);
        {
            var r = pathfinder.FindPath(new int2(1, 1), new int2(5, 5));
            Assert.IsTrue(r);
        }

        {
            var path = new NativeList<int2>(Allocator.Temp);
            var r = pathfinder.GetPath(path, new int2(5, 5));
            Assert.IsTrue(r);
            UnityEngine.Debug.Log(string.Join(", ", path.ToArray()));
            Assert.AreEqual(6, path.Length);
            Assert.AreEqual(new int2(5, 5), path[0]);
            Assert.AreEqual(new int2(5, 1), path[1]);
            Assert.AreEqual(new int2(3, 1), path[2]);
            Assert.AreEqual(new int2(3, 5), path[3]);
            Assert.AreEqual(new int2(1, 5), path[4]);
            Assert.AreEqual(new int2(1, 1), path[5]);
        }
    }

    [Test]
    public void ParseMap_ParsesSmallMap()
    {
        ParseMap(new []{ " "}, out var weights, out var dim);
        Assert.AreEqual(new int2(3, 3), dim);
        for (int i = 0; i < 9; i++)
        {
            if (i == 4)
                Assert.AreEqual(0, weights[i]);
            else
                Assert.AreEqual(float.PositiveInfinity, weights[i]);
        }
    }

    private static void ParseMap(string[] map, out NativeArray<float> weights, out int2 dim)
    {
        int h = map.Length + 2;
        int w = map[0].Length + 2;
        dim = new int2(w, h);
        weights = new NativeArray<float>(w * h, Allocator.Temp);
        for (int i = 0; i < w * h; i++)
            weights[i] = float.PositiveInfinity;

        for (int y = 1; y < h - 1; y++)
        {
            var l = map[y-1];
            for (int x = 1; x < w - 1; x++)
            {
                var c = l[x - 1];
                if (c == ' ')
                    weights[y * w + x] = 0;
                else if ('0' <= c && c <= '9')
                    weights[y * w + x] = (c - '0');
            }
        }
    }
}