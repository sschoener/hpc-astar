using System;
using System.Collections.Generic;
using NUnit.Framework;
using TowerDefense;
using Unity.Collections;

public class HeapTests
{
    [Test]
    public void Heap_IsEmpty_WhenEmpty()
    {
        using (var heap = new UnsafeHeap<int, DefaultComparer<int>>(1, Allocator.Temp))
        {
            Assert.IsTrue(heap.IsEmpty);
        }
    }

    [Test]
    public void Heap_Head_ReturnsMaximum([Values(1, 2, 3, 4, 5, 10)]int n)
    {
        using (var heap = new UnsafeHeap<int, DefaultComparer<int>>(4, Allocator.Temp))
        {
            // insert forward
            for (int i = 0; i < n; i++)
            {
                heap.Add(i);
                Assert.AreEqual(i, heap.Head);
                Assert.AreEqual(-1, heap.ValidateIntegrity());
            }
            Assert.AreEqual(n, heap.Count);
            Assert.AreEqual(-1, heap.ValidateIntegrity());

            heap.Clear();
            Assert.AreEqual(0, heap.Count);
            Assert.AreEqual(-1, heap.ValidateIntegrity());

            // insert backward
            for (int i = n - 1; i >= 0; i--)
            {
                heap.Add(i);
                Assert.AreEqual(n - 1, heap.Head);
                Assert.AreEqual(-1, heap.ValidateIntegrity());
            }
            Assert.AreEqual(n, heap.Count);
            Assert.AreEqual(-1, heap.ValidateIntegrity());

            // remove
            for (int i = 0; i < n; i++)
            {
                var h = heap.Head;
                var e = heap.ExtractHead();
                Assert.AreEqual(-1, heap.ValidateIntegrity());
                Assert.AreEqual(h, e);
                Assert.AreEqual(n - i - 1, e);
            }
            Assert.AreEqual(0, heap.Count);
        }
    }

    [Test]
    public void Heap_Insert_IncreasesCount()
    {
        using (var heap = new UnsafeHeap<int, DefaultComparer<int>>(4, Allocator.Temp))
        {
            heap.Add(0);
            Assert.AreEqual(1, heap.Count);
            heap.Add(1);
            Assert.AreEqual(2, heap.Count);
            heap.Add(2);
            Assert.AreEqual(3, heap.Count);
            heap.Add(3);
            Assert.AreEqual(4, heap.Count);
        }
    }

    [Test]
    public void Heap_Insert_WithCapacityExceeded_IncreasesCount()
    {
        using (var heap = new UnsafeHeap<int, DefaultComparer<int>>(1, Allocator.Temp))
        {
            heap.Add(0);
            Assert.AreEqual(1, heap.Count);
            heap.Add(1);
            Assert.AreEqual(2, heap.Count);
            heap.Add(2);
            Assert.AreEqual(3, heap.Count);
            heap.Add(3);
            Assert.AreEqual(4, heap.Count);
        }
    }

    [Test]
    public void Heap_Insert_CanUseSameValue()
    {
        using (var heap = new UnsafeHeap<int, DefaultComparer<int>>(1, Allocator.Temp))
        {
            heap.Add(0);
            Assert.AreEqual(1, heap.Count);
            heap.Add(0);
            Assert.AreEqual(2, heap.Count);
            heap.Add(2);
            Assert.AreEqual(3, heap.Count);
            heap.Add(2);
            Assert.AreEqual(4, heap.Count);
        }
    }

    struct DefaultComparer<T> : IComparer<T> where T : IComparable<T>
    {
        public int Compare(T x, T y) => x.CompareTo(y);
    }
}