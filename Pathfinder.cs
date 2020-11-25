using System;
using System.Collections.Generic;
using Unity.Assertions;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

namespace TowerDefense
{
    /// <summary>
    /// Simple A* based pathfinder for grids.
    /// 
    /// Usage: Give it a map of costs with a size.
    /// Important note: The map is assumed to have a 1 unit border that is impassable (cost = float.PositiveInfinity). This simplifies the core
    /// loop greatly because we never have to check for map borders (only when we start the search).
    /// 
    /// To get path, use either of the functions FloodFill or FindPath exposed below. These allow you to get the closest point to multiple starting locations on the map.
    /// Then call GetPath(outPath, position) to get a path to the given position from any of the starting positions.
    /// 
    /// </summary>
    public struct Pathfinder : IDisposable
    {
        private NativeArray<float> Cost;
        private readonly int2 Dimensions;
        private NativeArray<int> Parent;
        private NativeArray<float> Distance;
        private UnsafeHeap<Node, MinHeapComparer> Open;

        public Pathfinder(int2 dimensions, NativeArray<float> cost, Allocator allocator)
        {
            Cost = cost;
            Parent = new NativeArray<int>(dimensions.x * dimensions.y, allocator);
            Distance = new NativeArray<float>(dimensions.x * dimensions.y, allocator);
            Open = new UnsafeHeap<Node, MinHeapComparer>(1, allocator);
            Dimensions = dimensions;
        }
        
        
        readonly struct Node
        {
            public readonly ushort X, Y;
            public readonly int ParentIndex;
            public readonly float DistanceTravelled;
            public int2 Pos => new int2(X, Y);

            public Node(int2 p, int parentIndex, float d = 0)
            {
                X = (ushort) p.x;
                Y = (ushort) p.y;
                ParentIndex = parentIndex;
                DistanceTravelled = d;
            }
        }

        struct MinHeapComparer : IComparer<Node>
        {
            public int2 Target;

            public int Compare(Node x, Node y)
            {
                float dy = y.DistanceTravelled + math.lengthsq(Target - y.Pos);
                float dx = x.DistanceTravelled + math.lengthsq(Target - x.Pos);
                return -dx.CompareTo(dy);
            }
        }

        unsafe bool FillMap(int2* startingLocations, int count, int2? target)
        {
            UnsafeUtility.MemSet(Parent.GetUnsafePtr(), 0xFF, Parent.Length * sizeof(int));
            int w = Dimensions.x;
            int targetIdx = -1;
            if (target != null)
            {
                var t = target.Value;
                if (t.x <= 0 || t.x >= w - 1 || t.y <= 0 || t.y >= Dimensions.y)
                    return false;
                targetIdx = t.y * w + t.x;
                for (int i = 0; i < count; i++)
                {
                    if (startingLocations[i].Equals(t))
                    {
                        Parent[targetIdx] = startingLocations[i].y * w + startingLocations[i].x;
                        return true;
                    }
                }
            }


            Open.Clear();
            Open.Comparer.Target = target.GetValueOrDefault();

            for (int i = 0; i < count; i++)
            {
                var s = startingLocations[i];
                if (s.x <= 0 || s.x >= w - 1 || s.y <= 0 || s.y >= Dimensions.y)
                    return false;
                Open.Add(new Node(new int2(s.x, s.y), -1));
                var head = Open.Head;
            }

            int up = w;
            const int right = 1;
            int* neighborIndexDelta = stackalloc int[8] {
                up,
                -up,
                right,
                -right,
                up + right,
                up - right,
                -up + right,
                -up - right
            };
            int2* neighborDelta = stackalloc int2[8]
            {
                new int2(0, 1),
                new int2(0, -1),
                new int2(1, 0),
                new int2(-1, 0),
                new int2(1, 1),
                new int2(-1, 1),
                new int2(1, -1),
                new int2(-1, -1),
            };

            while (!Open.IsEmpty)
            {
                var h = Open.ExtractHead();
                int idx = h.Y * w + h.X;
                // have we already found a shorter path?
                if (Parent[idx] != -1 && Distance[idx] < h.DistanceTravelled)
                    continue;
                Parent[idx] = h.ParentIndex;
                Distance[idx] = h.DistanceTravelled;
                for (int i = 0; i < 8; i++)
                {
                    int neighborIdx = idx + neighborIndexDelta[i];
                    float cost = Cost[neighborIdx];
                    if (i >= 4)
                    {
                        var hCheckIndexDelta = neighborDelta[i].x;
                        cost += Cost[idx + hCheckIndexDelta] / 3;

                        var vCheckIndexDelta = neighborDelta[i].y == -1 ? -up : up;
                        cost += Cost[idx + vCheckIndexDelta] / 3;
                    }

                    if (neighborIdx == targetIdx && (i < 4 || cost < float.PositiveInfinity))
                    {
                        // TODO check for diagonal neighbors here?
                        Parent[targetIdx] = idx;
                        return true;
                    }

                    if (cost < float.PositiveInfinity)
                    {
                        cost += i >= 4 ? math.SQRT2 : 1;
                        Open.Add(new Node(h.Pos + neighborDelta[i], idx, h.DistanceTravelled + cost));
                    }
                }
            }

            return false;
        }

        public unsafe bool FloodFill(NativeArray<int2> startingLocations) => FillMap((int2*)startingLocations.GetUnsafeReadOnlyPtr(), startingLocations.Length, null);
        public unsafe bool FindPath(NativeArray<int2> startingLocations, int2 target) => FillMap((int2*)startingLocations.GetUnsafeReadOnlyPtr(), startingLocations.Length, target);
        public unsafe bool FindPath(int2 startingLocation, int2 target) => FillMap(&startingLocation, 1, target);

        public bool GetPath(NativeList<int2> path, int2 target)
        {
            int idx = target.y * Dimensions.x + target.x;
            int p = Parent[idx];
            if (p == -1)
                return false;
            path.Add(target);
            int2 lastPos = target;
            int2? lastDelta = null;
            while (p != -1)
            {
                int x = p % Dimensions.x;
                int y = p / Dimensions.x;
                var newPos = new int2(x, y);
                var newDelta = lastPos - newPos;
                if (lastDelta == null)
                    lastDelta = newDelta;
                else if (!lastDelta.Value.Equals(newDelta))
                {
                    path.Add(lastPos);
                    lastDelta = newDelta;
                }
                p = Parent[p];
                lastPos = newPos;
            }
            path.Add(lastPos);
            return true;
        }

        public void Dispose()
        {
            Cost.Dispose();
            Parent.Dispose();
            Distance.Dispose();
            Open.Dispose();
        }
    }

    public unsafe struct UnsafeHeap<T, U> : IDisposable
        where T : unmanaged
        where U : struct, IComparer<T>
    {
        public UnsafeList<T> _Buffer;
        public U Comparer;

        public UnsafeHeap(int capacity, U comparer, Allocator allocator)
        {
            _Buffer = new UnsafeList<T>(capacity, allocator);
            Comparer = comparer;
        }

        public UnsafeHeap(int capacity, Allocator allocator) : this(capacity, default, allocator)
        {}

        public int Count => _Buffer.Length;
        public bool IsEmpty => _Buffer.Length == 0;

        public void Add(T element)
        {
            _Buffer.Add(element);
            HeapHelpers.SiftUp<T, U>(_Buffer.Ptr, _Buffer.Length, 0, Comparer);
        }

        public void Clear()
        {
            _Buffer.Length = 0;
        }

        public T ExtractHead()
        {
            var h = _Buffer[0];
            _Buffer.RemoveAtSwapBack(0);
            HeapHelpers.Heapify<T, U>(_Buffer.Ptr, 1, _Buffer.Length, 0, Comparer);
            return h;
        }

        public T Head => _Buffer[0];

        public void Dispose()
        {
            _Buffer.Dispose();
        }

        public int ValidateIntegrity()
        {
            int n = Count;
            var comp = new U();
            for (int i = 1; i < n; i++)
            {
                int parent = (i + 1) / 2;
                if (comp.Compare(UnsafeUtility.ReadArrayElement<T>(_Buffer.Ptr, parent - 1),
                    UnsafeUtility.ReadArrayElement<T>(_Buffer.Ptr, i)) < 0)
                    return i;
            }

            return -1;
        }
    }

    static unsafe class HeapHelpers
    {
        public static void HeapSort<T, U>(void* array, int lo, int hi, U comp)
            where T : struct
            where U : IComparer<T>
        {
            int n = hi - lo + 1;

            for (int i = n / 2; i >= 1; i--)
            {
                Heapify<T, U>(array, i, n, lo, comp);
            }

            for (int i = n; i > 1; i--)
            {
                Swap<T>(array, lo, lo + i - 1);
                Heapify<T, U>(array, 1, i - 1, lo, comp);
            }
        }

        public static void SiftUp<T, U>(void* array, int i, int lo, U comp)
            where T : struct where U : IComparer<T>
        {
            T val = UnsafeUtility.ReadArrayElement<T>(array, lo + i - 1);
            while (i > 1)
            {
                int parent = i / 2;
                if (comp.Compare(UnsafeUtility.ReadArrayElement<T>(array, lo + parent - 1), val) >= 0)
                    break;
                // move parent into our place
                UnsafeUtility.WriteArrayElement(array, lo + i - 1, UnsafeUtility.ReadArrayElement<T>(array, lo + parent - 1));
                i = parent;
            }
            UnsafeUtility.WriteArrayElement(array, lo + i - 1, val);
        }

        static void Swap<T>(void* array, int lhs, int rhs) where T : struct
        {
            T val = UnsafeUtility.ReadArrayElement<T>(array, lhs);
            UnsafeUtility.WriteArrayElement(array, lhs, UnsafeUtility.ReadArrayElement<T>(array, rhs));
            UnsafeUtility.WriteArrayElement(array, rhs, val);
        }

        public static void Heapify<T, U>(void* array, int i, int n, int lo, U comp) where T : struct where U : IComparer<T>
        {
            T val = UnsafeUtility.ReadArrayElement<T>(array, lo + i - 1);
            int child;
            while (i <= n / 2)
            {
                // find the smaller of the two children
                child = 2 * i;
                if (child < n && (comp.Compare(UnsafeUtility.ReadArrayElement<T>(array, lo + child - 1), UnsafeUtility.ReadArrayElement<T>(array, (lo + child))) < 0))
                {
                    child++;
                }
                // if we are smaller than our children, we're at the right place
                if (comp.Compare(UnsafeUtility.ReadArrayElement<T>(array, (lo + child - 1)), val) < 0)
                    break;

                // otherwise, swap with the smaller child and recurse.
                UnsafeUtility.WriteArrayElement(array, lo + i - 1, UnsafeUtility.ReadArrayElement<T>(array, lo + child - 1));
                i = child;
            }
            UnsafeUtility.WriteArrayElement(array, lo + i - 1, val);
        }
    }
}