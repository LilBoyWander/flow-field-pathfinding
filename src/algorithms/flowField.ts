import { MinHeap } from './minHeap';
import { GridMap, TERRAIN_WALL } from '../simulation/gridMap';

export interface FlowFieldBuildStats {
  duration: number;
  expandedNodes: number;
  reachableCells: number;
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
 * Builds one integration field from the destination, then derives a normalized direction vector for every cell.
 * All agents can reuse those vectors until the goal or terrain changes.
 */
export class FlowField {
  readonly integration: Float32Array;
  readonly vectors: Float32Array;
  private readonly frontier = new MinHeap();
  private readonly map: GridMap;

  constructor(map: GridMap) {
    this.map = map;
    this.integration = new Float32Array(map.size);
    this.vectors = new Float32Array(map.size * 2);
  }

  build(goalIndex: number): FlowFieldBuildStats {
    const startedAt = performance.now();
    this.integration.fill(Number.POSITIVE_INFINITY);
    this.vectors.fill(0);
    this.frontier.clear();

    this.integration[goalIndex] = 0;
    this.frontier.push(goalIndex, 0);

    let expandedNodes = 0;
    while (this.frontier.size > 0) {
      const entry = this.frontier.pop();
      if (!entry || entry.priority !== this.integration[entry.node]) {
        continue;
      }

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
        if (terrainCost === TERRAIN_WALL || !this.canMoveDiagonally(column, row, direction.x, direction.y)) {
          continue;
        }

        // Integration lives in Float32Array, so round the queued priority to the exact stored representation.
        // Without this, diagonal costs can look stale when their JavaScript double is compared with the float value.
        const nextCost = Math.fround(entry.priority + terrainCost * direction.distance);
        if (nextCost < this.integration[nextIndex]) {
          this.integration[nextIndex] = nextCost;
          this.frontier.push(nextIndex, nextCost);
        }
      }
    }

    let reachableCells = 0;
    for (let index = 0; index < this.map.size; index += 1) {
      if (!Number.isFinite(this.integration[index])) {
        continue;
      }

      reachableCells += 1;
      if (index === goalIndex || this.map.costs[index] === TERRAIN_WALL) {
        continue;
      }

      const column = this.map.getColumn(index);
      const row = this.map.getRow(index);
      let bestCost = this.integration[index];
      let bestX = 0;
      let bestY = 0;

      for (const direction of DIRECTIONS) {
        const nextColumn = column + direction.x;
        const nextRow = row + direction.y;
        if (
          !this.map.inBounds(nextColumn, nextRow) ||
          !this.canMoveDiagonally(column, row, direction.x, direction.y)
        ) {
          continue;
        }

        const nextIndex = this.map.getIndex(nextColumn, nextRow);
        if (this.integration[nextIndex] < bestCost) {
          bestCost = this.integration[nextIndex];
          bestX = direction.x;
          bestY = direction.y;
        }
      }

      const length = Math.hypot(bestX, bestY) || 1;
      this.vectors[index * 2] = bestX / length;
      this.vectors[index * 2 + 1] = bestY / length;
    }

    return {
      duration: performance.now() - startedAt,
      expandedNodes,
      reachableCells,
    };
  }

  private canMoveDiagonally(column: number, row: number, offsetX: number, offsetY: number): boolean {
    if (offsetX === 0 || offsetY === 0) {
      return true;
    }

    const horizontal = this.map.getIndex(column + offsetX, row);
    const vertical = this.map.getIndex(column, row + offsetY);
    return this.map.isWalkableIndex(horizontal) && this.map.isWalkableIndex(vertical);
  }
}
