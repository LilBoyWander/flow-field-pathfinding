import type { FlowField } from '../algorithms/flowField';
import type { GridMap } from './gridMap';

export type NavigationMode = 'flow' | 'astar';

export interface Agent {
  x: number;
  y: number;
  velocityX: number;
  velocityY: number;
  radius: number;
  path: Int32Array;
  pathIndex: number;
  reached: boolean;
  tint: number;
}

/**
 * Crowd movement intentionally stays separate from path construction.
 *
 * Flow-field agents sample one shared vector. A* agents follow their own waypoint arrays. Local collision avoidance is
 * left out so the case study measures global path planning rather than mixing two different navigation problems.
 */
export class Crowd {
  readonly agents: Agent[] = [];
  private randomState = 0x5f3759df;
  private readonly map: GridMap;
  private readonly cellSize: number;

  constructor(map: GridMap, cellSize: number) {
    this.map = map;
    this.cellSize = cellSize;
  }

  setCount(count: number): void {
    while (this.agents.length < count) {
      this.agents.push(this.createAgent());
    }
    if (this.agents.length > count) {
      this.agents.length = count;
    }
  }

  resetPositions(): void {
    for (const agent of this.agents) {
      this.placeAgent(agent);
    }
  }

  clearPaths(): void {
    for (const agent of this.agents) {
      agent.path = new Int32Array();
      agent.pathIndex = 0;
      agent.reached = false;
    }
  }

  getAgentCellIndex(agent: Agent): number {
    const column = Math.max(0, Math.min(this.map.columns - 1, Math.floor(agent.x / this.cellSize)));
    const row = Math.max(0, Math.min(this.map.rows - 1, Math.floor(agent.y / this.cellSize)));
    return this.map.getIndex(column, row);
  }

  update(
    deltaTime: number,
    mode: NavigationMode,
    goalIndex: number,
    flowField: FlowField,
    speedMultiplier: number,
  ): number {
    const goalX = (this.map.getColumn(goalIndex) + 0.5) * this.cellSize;
    const goalY = (this.map.getRow(goalIndex) + 0.5) * this.cellSize;
    let reachedCount = 0;

    for (const agent of this.agents) {
      if (agent.reached) {
        reachedCount += 1;
        continue;
      }

      let directionX = 0;
      let directionY = 0;
      if (mode === 'flow') {
        const cellIndex = this.getAgentCellIndex(agent);
        directionX = flowField.vectors[cellIndex * 2];
        directionY = flowField.vectors[cellIndex * 2 + 1];
      } else if (agent.path.length > 0 && agent.pathIndex < agent.path.length) {
        const waypoint = agent.path[agent.pathIndex];
        const waypointX = (this.map.getColumn(waypoint) + 0.5) * this.cellSize;
        const waypointY = (this.map.getRow(waypoint) + 0.5) * this.cellSize;
        const distance = Math.hypot(waypointX - agent.x, waypointY - agent.y);
        if (distance < this.cellSize * 0.42 && agent.pathIndex < agent.path.length - 1) {
          agent.pathIndex += 1;
        }

        const activeWaypoint = agent.path[agent.pathIndex];
        const activeX = (this.map.getColumn(activeWaypoint) + 0.5) * this.cellSize;
        const activeY = (this.map.getRow(activeWaypoint) + 0.5) * this.cellSize;
        const length = Math.hypot(activeX - agent.x, activeY - agent.y) || 1;
        directionX = (activeX - agent.x) / length;
        directionY = (activeY - agent.y) / length;
      }

      const maxSpeed = 48 * speedMultiplier;
      agent.velocityX = agent.velocityX * 0.82 + directionX * maxSpeed * 0.18;
      agent.velocityY = agent.velocityY * 0.82 + directionY * maxSpeed * 0.18;

      const velocityLength = Math.hypot(agent.velocityX, agent.velocityY);
      if (velocityLength > maxSpeed) {
        agent.velocityX = (agent.velocityX / velocityLength) * maxSpeed;
        agent.velocityY = (agent.velocityY / velocityLength) * maxSpeed;
      }

      const nextX = agent.x + agent.velocityX * deltaTime;
      const nextY = agent.y + agent.velocityY * deltaTime;
      const nextColumn = Math.floor(nextX / this.cellSize);
      const nextRow = Math.floor(nextY / this.cellSize);
      if (
        this.map.inBounds(nextColumn, nextRow) &&
        this.map.isWalkableIndex(this.map.getIndex(nextColumn, nextRow))
      ) {
        agent.x = nextX;
        agent.y = nextY;
      } else {
        agent.velocityX *= -0.15;
        agent.velocityY *= -0.15;
      }

      if (Math.hypot(goalX - agent.x, goalY - agent.y) < this.cellSize * 0.72) {
        agent.reached = true;
        agent.velocityX = 0;
        agent.velocityY = 0;
        reachedCount += 1;
      }
    }

    return reachedCount;
  }

  private createAgent(): Agent {
    const agent: Agent = {
      x: 0,
      y: 0,
      velocityX: 0,
      velocityY: 0,
      radius: 2.25 + this.random() * 1.25,
      path: new Int32Array(),
      pathIndex: 0,
      reached: false,
      tint: this.random(),
    };
    this.placeAgent(agent);
    return agent;
  }

  private placeAgent(agent: Agent): void {
    const spawnColumns = Math.max(4, Math.floor(this.map.columns * 0.3));
    for (let attempt = 0; attempt < 200; attempt += 1) {
      const column = 1 + Math.floor(this.random() * (spawnColumns - 2));
      const row = 1 + Math.floor(this.random() * (this.map.rows - 2));
      const index = this.map.getIndex(column, row);
      if (!this.map.isWalkableIndex(index)) {
        continue;
      }

      agent.x = (column + 0.18 + this.random() * 0.64) * this.cellSize;
      agent.y = (row + 0.18 + this.random() * 0.64) * this.cellSize;
      agent.velocityX = 0;
      agent.velocityY = 0;
      agent.path = new Int32Array();
      agent.pathIndex = 0;
      agent.reached = false;
      return;
    }
  }

  private random(): number {
    this.randomState += 0x6d2b79f5;
    let value = this.randomState;
    value = Math.imul(value ^ (value >>> 15), value | 1);
    value ^= value + Math.imul(value ^ (value >>> 7), value | 61);
    return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
  }
}
