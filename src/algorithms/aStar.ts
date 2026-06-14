import { MinHeap } from './minHeap';
import { GridMap, TERRAIN_WALL } from '../simulation/gridMap';

export interface AStarResult {
  path: Int32Array;
  expandedNodes: number;
  cost: number;
}

const DIRECTIONS = [
  { x: 0, y: -1, distance: 1 },
  { x: 1, y: 0, distance: 1 },
  { x: 0, y: 1, distance: 1 },
  { x: -1, y: 0, distance: 1 },
  { x: 1, y: -1, distance: Math.SQRT2 },
  { x: 1, y: 1, distance: Math.SQRT2 },
  { x: -1, y: 1, distance: Math.SQRT2 },
  { x: -1, y: -1, distance: Math.SQRT2 },
] as const;

/**
 * Independent A* planner with reusable scratch memory.
 *
 * Every agent still performs its own search. Reusing buffers removes allocation noise without hiding that algorithmic
 * difference from the comparison.
 */
export class AStarPlanner {
  private readonly open = new MinHeap();
  private readonly gScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly closed: Uint8Array;
  private readonly map: GridMap;

  constructor(map: GridMap) {
    this.map = map;
    this.gScore = new Float32Array(map.size);
    this.cameFrom = new Int32Array(map.size);
    this.closed = new Uint8Array(map.size);
  }

  findPath(startIndex: number, goalIndex: number): AStarResult {
    if (!this.map.isWalkableIndex(startIndex) || !this.map.isWalkableIndex(goalIndex)) {
      return { path: new Int32Array(), expandedNodes: 0, cost: Number.POSITIVE_INFINITY };
    }

    this.gScore.fill(Number.POSITIVE_INFINITY);
    this.cameFrom.fill(-1);
    this.closed.fill(0);
    this.open.clear();

    this.gScore[startIndex] = 0;
    this.open.push(startIndex, this.heuristic(startIndex, goalIndex));

    let expandedNodes = 0;
    while (this.open.size > 0) {
      const entry = this.open.pop();
      if (!entry || this.closed[entry.node] === 1) {
        continue;
      }

      if (entry.node === goalIndex) {
        // gScore is the comparable contract value. Path reconstruction is useful for agents,
        // but the audit compares this accumulated terrain cost with flow integration.
        return {
          path: this.reconstructPath(startIndex, goalIndex),
          expandedNodes,
          cost: this.gScore[goalIndex],
        };
      }

      this.closed[entry.node] = 1;
      expandedNodes += 1;
      const column = this.map.getColumn(entry.node);
      const row = this.map.getRow(entry.node);

      for (const direction of DIRECTIONS) {
        const nextColumn = column + direction.x;
        const nextRow = row + direction.y;
        if (!this.map.inBounds(nextColumn, nextRow)) {
          continue;
        }

        const nextIndex = this.map.getIndex(nextColumn, nextRow);
        const terrainCost = this.map.costs[nextIndex];
        if (
          terrainCost === TERRAIN_WALL ||
          this.closed[nextIndex] === 1 ||
          !this.canMoveDiagonally(column, row, direction.x, direction.y)
        ) {
          continue;
        }

        const nextScore = this.gScore[entry.node] + terrainCost * direction.distance;
        if (nextScore < this.gScore[nextIndex]) {
          this.gScore[nextIndex] = nextScore;
          this.cameFrom[nextIndex] = entry.node;
          this.open.push(nextIndex, nextScore + this.heuristic(nextIndex, goalIndex));
        }
      }
    }

    return { path: new Int32Array(), expandedNodes, cost: Number.POSITIVE_INFINITY };
  }

  private reconstructPath(startIndex: number, goalIndex: number): Int32Array {
    const reversed: number[] = [goalIndex];
    let current = goalIndex;

    while (current !== startIndex) {
      current = this.cameFrom[current];
      if (current < 0) {
        return new Int32Array();
      }
      reversed.push(current);
    }

    reversed.reverse();
    return Int32Array.from(reversed);
  }

  private heuristic(index: number, goalIndex: number): number {
    const deltaX = Math.abs(this.map.getColumn(index) - this.map.getColumn(goalIndex));
    const deltaY = Math.abs(this.map.getRow(index) - this.map.getRow(goalIndex));
    const diagonal = Math.min(deltaX, deltaY);
    // Octile distance assumes the cheapest terrain cost (one), so it remains admissible when
    // rough cells cost more and A* returns the same optimum encoded by the flow field.
    return deltaX + deltaY + (Math.SQRT2 - 2) * diagonal;
  }

  private canMoveDiagonally(column: number, row: number, offsetX: number, offsetY: number): boolean {
    if (offsetX === 0 || offsetY === 0) {
      return true;
    }

    return (
      this.map.isWalkableIndex(this.map.getIndex(column + offsetX, row)) &&
      this.map.isWalkableIndex(this.map.getIndex(column, row + offsetY))
    );
  }
}
