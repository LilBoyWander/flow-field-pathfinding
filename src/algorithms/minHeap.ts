interface HeapEntry {
  node: number;
  priority: number;
}

/**
 * Small reusable binary min-heap used by both navigation algorithms.
 *
 * Duplicate entries are allowed. Callers discard stale entries when they are popped, which keeps decrease-key logic
 * out of the hot path and makes the implementation easier to audit.
 */
export class MinHeap {
  private readonly entries: HeapEntry[] = [];

  get size(): number {
    return this.entries.length;
  }

  clear(): void {
    this.entries.length = 0;
  }

  push(node: number, priority: number): void {
    const entry = { node, priority };
    this.entries.push(entry);

    let index = this.entries.length - 1;
    while (index > 0) {
      const parentIndex = Math.floor((index - 1) / 2);
      const parent = this.entries[parentIndex];
      if (parent.priority <= priority) {
        break;
      }

      this.entries[index] = parent;
      index = parentIndex;
    }

    this.entries[index] = entry;
  }

  pop(): HeapEntry | undefined {
    if (this.entries.length === 0) {
      return undefined;
    }

    const root = this.entries[0];
    const tail = this.entries.pop();

    if (this.entries.length === 0 || !tail) {
      return root;
    }

    let index = 0;
    while (true) {
      const leftIndex = index * 2 + 1;
      const rightIndex = leftIndex + 1;
      if (leftIndex >= this.entries.length) {
        break;
      }

      let childIndex = leftIndex;
      if (
        rightIndex < this.entries.length &&
        this.entries[rightIndex].priority < this.entries[leftIndex].priority
      ) {
        childIndex = rightIndex;
      }

      if (this.entries[childIndex].priority >= tail.priority) {
        break;
      }

      this.entries[index] = this.entries[childIndex];
      index = childIndex;
    }

    this.entries[index] = tail;
    return root;
  }
}
