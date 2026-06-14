export type TerrainPreset = 'open' | 'chokepoints' | 'maze' | 'islands';
export type PaintTool = 'goal' | 'wall' | 'rough' | 'erase';

export const TERRAIN_WALL = 0;
export const TERRAIN_NORMAL = 1;
export const TERRAIN_ROUGH = 4;

/** Deterministic pseudo-random generator so scenario presets remain reproducible. */
function createRandom(seed: number): () => number {
  let value = seed >>> 0;
  return () => {
    value += 0x6d2b79f5;
    let result = value;
    result = Math.imul(result ^ (result >>> 15), result | 1);
    result ^= result + Math.imul(result ^ (result >>> 7), result | 61);
    return ((result ^ (result >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * Compact cost grid shared by the flow-field and A* planners.
 *
 * Zero is blocked, one is normal terrain, and larger values are traversable but more expensive.
 */
export class GridMap {
  readonly costs: Uint8Array;
  readonly columns: number;
  readonly rows: number;

  constructor(columns: number, rows: number) {
    this.columns = columns;
    this.rows = rows;
    this.costs = new Uint8Array(columns * rows);
    this.costs.fill(TERRAIN_NORMAL);
  }

  get size(): number {
    return this.costs.length;
  }

  getWalkableCount(): number {
    let count = 0;
    for (const cost of this.costs) {
      if (cost !== TERRAIN_WALL) {
        count += 1;
      }
    }
    return count;
  }

  getIndex(column: number, row: number): number {
    return row * this.columns + column;
  }

  getColumn(index: number): number {
    return index % this.columns;
  }

  getRow(index: number): number {
    return Math.floor(index / this.columns);
  }

  inBounds(column: number, row: number): boolean {
    return column >= 0 && column < this.columns && row >= 0 && row < this.rows;
  }

  isWalkableIndex(index: number): boolean {
    return index >= 0 && index < this.size && this.costs[index] !== TERRAIN_WALL;
  }

  setCost(column: number, row: number, cost: number): void {
    if (this.inBounds(column, row)) {
      this.costs[this.getIndex(column, row)] = cost;
    }
  }

  applyBrush(column: number, row: number, radius: number, tool: PaintTool): void {
    const cost = tool === 'wall' ? TERRAIN_WALL : tool === 'rough' ? TERRAIN_ROUGH : TERRAIN_NORMAL;

    for (let offsetY = -radius + 1; offsetY < radius; offsetY += 1) {
      for (let offsetX = -radius + 1; offsetX < radius; offsetX += 1) {
        const nextColumn = column + offsetX;
        const nextRow = row + offsetY;
        if (this.inBounds(nextColumn, nextRow)) {
          this.setCost(nextColumn, nextRow, cost);
        }
      }
    }
  }

  /**
   * Presets create repeatable workloads instead of random screenshots that cannot be meaningfully compared.
   */
  loadPreset(preset: TerrainPreset): void {
    this.costs.fill(TERRAIN_NORMAL);

    if (preset === 'open') {
      return;
    }

    if (preset === 'chokepoints') {
      const firstWall = Math.floor(this.columns * 0.36);
      const secondWall = Math.floor(this.columns * 0.66);
      for (let row = 0; row < this.rows; row += 1) {
        if (row < 8 || row > 14) {
          this.setCost(firstWall, row, TERRAIN_WALL);
        }
        if (row < 25 || row > 31) {
          this.setCost(secondWall, row, TERRAIN_WALL);
        }
      }

      for (let column = 6; column < 18; column += 1) {
        for (let row = 27; row < 35; row += 1) {
          this.setCost(column, row, TERRAIN_ROUGH);
        }
      }
      return;
    }

    if (preset === 'maze') {
      for (let column = 8; column < this.columns - 7; column += 8) {
        const gapStart = column % 16 === 0 ? 5 : this.rows - 10;
        for (let row = 3; row < this.rows - 3; row += 1) {
          if (row < gapStart || row > gapStart + 5) {
            this.setCost(column, row, TERRAIN_WALL);
          }
        }
      }
      return;
    }

    const random = createRandom(20260613);
    for (let island = 0; island < 22; island += 1) {
      const centerColumn = 8 + Math.floor(random() * (this.columns - 16));
      const centerRow = 4 + Math.floor(random() * (this.rows - 8));
      const radius = 2 + Math.floor(random() * 4);
      const terrain = island % 4 === 0 ? TERRAIN_ROUGH : TERRAIN_WALL;

      for (let row = centerRow - radius; row <= centerRow + radius; row += 1) {
        for (let column = centerColumn - radius; column <= centerColumn + radius; column += 1) {
          if (
            this.inBounds(column, row) &&
            Math.hypot(column - centerColumn, row - centerRow) <= radius + random() * 0.8
          ) {
            this.setCost(column, row, terrain);
          }
        }
      }
    }
  }

  findNearestWalkable(column: number, row: number): number {
    const clampedColumn = Math.max(0, Math.min(this.columns - 1, column));
    const clampedRow = Math.max(0, Math.min(this.rows - 1, row));
    const origin = this.getIndex(clampedColumn, clampedRow);
    if (this.isWalkableIndex(origin)) {
      return origin;
    }

    for (let radius = 1; radius < Math.max(this.columns, this.rows); radius += 1) {
      for (let offsetY = -radius; offsetY <= radius; offsetY += 1) {
        for (let offsetX = -radius; offsetX <= radius; offsetX += 1) {
          if (Math.abs(offsetX) !== radius && Math.abs(offsetY) !== radius) {
            continue;
          }

          const nextColumn = clampedColumn + offsetX;
          const nextRow = clampedRow + offsetY;
          if (!this.inBounds(nextColumn, nextRow)) {
            continue;
          }

          const index = this.getIndex(nextColumn, nextRow);
          if (this.isWalkableIndex(index)) {
            return index;
          }
        }
      }
    }

    return origin;
  }
}
