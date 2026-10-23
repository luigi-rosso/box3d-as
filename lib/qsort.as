/*
 * Copyright (c) 2013, 2017 Alexey Tourbin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// AnimaScript port of src/qsort.h

/*
 * This is a traditional Quicksort implementation which mostly follows
 * [Sedgewick 1978].  Sorting is performed entirely on array indices,
 * while actual access to the array elements is abstracted out with the
 * user-defined `LESS` and `SWAP` primitives.
 *
 * Synopsis:
 *	QSORT(N, LESS, SWAP);
 * where
 *	N - the number of elements in A[];
 *	LESS(i, j) - compares A[i] to A[j];
 *	SWAP(i, j) - exchanges A[i] with A[j].
 *
 * Q_LOOP<S> takes the primitives as the methods less(i, j) and swap(i, j) of
 * a class S, so a hot call site gets direct calls; QSORT takes closures.
 */

// Indices stay i32: the macro's unsigned long indices never wrap, so the
// comparisons are the same.

/* Sort 3 elements. */
@inline function Q_SORT3<S>(q_a1: i32, q_a2: i32, q_a3: i32, s: S): void {
  if (s.less(q_a2, q_a1)) {
    if (s.less(q_a3, q_a2)) {
      s.swap(q_a1, q_a3);
    } else {
      s.swap(q_a1, q_a2);
      if (s.less(q_a3, q_a2)) {
        s.swap(q_a2, q_a3);
      }
    }
  } else if (s.less(q_a3, q_a2)) {
    s.swap(q_a2, q_a3);
    if (s.less(q_a2, q_a1)) {
      s.swap(q_a1, q_a2);
    }
  }
}

/* When the size of [q_l,q_r], i.e. q_r-q_l+1, is greater than or equal to
 * Q_THRESH, the algorithm performs recursive partitioning.  When the size
 * drops below Q_THRESH, the algorithm switches to insertion sort.
 * The minimum valid value is probably 5 (with 5 items, the second and
 * the middle items, the middle itself being rounded down, are distinct). */
export const Q_THRESH: i32 = 16;

// The C stack holds 32 (q_l, q_r) frames when Q_N is a 32 bit int.
const q_st = new StaticArray<i32>(2 * 32);

/* The main loop. */
export function Q_LOOP<S>(Q_N: i32, s: S): void {
  let q_l = 0;
  let q_r = Q_N - 1;
  let q_sp = 0; /* the number of frames pushed to the stack */
  while (1) {
    if (q_r - q_l + 1 >= Q_THRESH) {
      let q_i = 0;
      let q_j = 0;
      // Q_PARTITION(q_l, q_r, q_i, q_j, Q_UINT, Q_LESS, Q_SWAP)
      /* Partition [q_l,q_r] around a pivot.  After partitioning,
       * [q_l,q_j] are the elements that are less than or equal to the pivot,
       * while [q_i,q_r] are the elements greater than or equal to the pivot. */
      {
        /* The middle element, not to be confused with the median. */
        let q_m = q_l + ((q_r - q_l) >> 1);
        /* Reorder the second, the middle, and the last items.
         * As [Edelkamp Weiss 2016] explain, using the second element
         * instead of the first one helps avoid bad behaviour for
         * decreasingly sorted arrays.  This method is used in recent
         * versions of gcc's std::sort, see gcc bug 58437#c13, although
         * the details are somewhat different (cf. #c14). */
        Q_SORT3<S>(q_l + 1, q_m, q_r, s);
        /* Place the median at the beginning. */
        s.swap(q_l, q_m);
        /* Partition [q_l+2, q_r-1] around the median which is in q_l.
         * q_i and q_j are initially off by one, they get decremented
         * in the do-while loops. */
        q_i = q_l + 1;
        q_j = q_r;
        while (1) {
          do {
            q_i++;
          } while (s.less(q_i, q_l));
          do {
            q_j--;
          } while (s.less(q_l, q_j));
          if (q_i >= q_j) break; /* Sedgewick says "until j < i" */
          s.swap(q_i, q_j);
        }
        /* Compensate for the i==j case. */
        q_i = q_j + 1;
        /* Put the median to its final place. */
        s.swap(q_l, q_j);
        /* The median is not part of the left subfile. */
        q_j--;
      }
      /* Now have two subfiles: [q_l,q_j] and [q_i,q_r].
       * Dealing with them depends on which one is bigger. */
      let q_l1 = q_l;
      let q_r1 = q_j;
      let q_l2 = q_i;
      let q_r2 = q_r;
      if (!(q_j - q_l >= q_r - q_i)) {
        q_l1 = q_i;
        q_r1 = q_r;
        q_l2 = q_l;
        q_r2 = q_j;
      }
      // Q_SUBFILES(q_l1, q_r1, q_l2, q_r2)
      /* The missing part: dealing with subfiles.
       * Assumes that the first subfile is not smaller than the second. */
      /* If the second subfile is only a single element, it needs
       * no further processing.  The first subfile will be processed
       * on the next iteration (both subfiles cannot be only a single
       * element, due to Q_THRESH). */
      if (q_l2 == q_r2) {
        q_l = q_l1;
        q_r = q_r1;
      } else {
        /* Otherwise, both subfiles need processing.
         * Push the larger subfile onto the stack. */
        q_st[2 * q_sp] = q_l1;
        q_st[2 * q_sp + 1] = q_r1;
        q_sp++;
        /* Process the smaller subfile on the next iteration. */
        q_l = q_l2;
        q_r = q_r2;
      }
    } else {
      // Q_INSERTION_SORT(q_l, q_r, Q_UINT, Q_LESS, Q_SWAP)
      /* Insertion sort is applied to small subfiles - this is contrary to
       * Sedgewick's suggestion to run a separate insertion sort pass after
       * the partitioning is done.  The reason I don't like a separate pass
       * is that it triggers extra comparisons, because it can't see that the
       * medians are already in their final positions and need not be rechecked.
       * Since I do not assume that comparisons are cheap, I also do not try
       * to eliminate the (q_j > q_l) boundary check. */
      /* For each item starting with the second... */
      for (let q_i = q_l + 1; q_i <= q_r; q_i++) {
        /* move it down the array so that the first part is sorted. */
        for (let q_j = q_i; q_j > q_l && s.less(q_j, q_j - 1); q_j--) {
          s.swap(q_j, q_j - 1);
        }
      }
      /* Pop subfiles from the stack, until it gets empty. */
      if (q_sp == 0) break;
      q_sp--;
      q_l = q_st[2 * q_sp];
      q_r = q_st[2 * q_sp + 1];
    }
  }
}

export type Q_LESS = (i: i32, j: i32) => bool;
export type Q_SWAP = (i: i32, j: i32) => void;

class b3ClosureSorter {
  constructor(public lessFn: Q_LESS, public swapFn: Q_SWAP) {}
  @inline less(i: i32, j: i32): bool {
    return this.lessFn(i, j);
  }
  @inline swap(i: i32, j: i32): void {
    this.swapFn(i, j);
  }
}

/* And now, ladies and gentlemen, may I proudly present to you... */
export function QSORT(Q_N: i32, LESS: Q_LESS, SWAP: Q_SWAP): void {
  if (Q_N > 1) {
    /* We could check sizeof(Q_N) and use "unsigned", but at least
     * on x86_64, this has the performance penalty of up to 5%. */
    Q_LOOP<b3ClosureSorter>(Q_N, new b3ClosureSorter(LESS, SWAP));
  }
}
