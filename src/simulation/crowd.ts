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
 * Flow-field agents sample one shared vector. A* agents follow their own waypoint arrays. A lightweight dynamic
 * occupancy layer adds local separation without changing either planner or entering planner-build measurements.
 */
export class Crowd {
  readonly agents: Agent[] = [];
  private randomState = 0x5f3759df;
  private readonly map: GridMap;
  private readonly cellSize: number;
  private readonly occupancy: Uint16Array;
  private readonly bucketHeads: Int32Array;
  private nextAgent = new Int32Array();
  private maximumOccupancy = 0;

  constructor(map: GridMap, cellSize: number) {
    this.map = map;
    this.cellSize = cellSize;
    this.occupancy = new Uint16Array(map.size);
    this.bucketHeads = new Int32Array(map.size);
    this.bucketHeads.fill(-1);
  }

  setCount(count: number): void {
    while (this.agents.length < count) {
      this.agents.push(this.createAgent());
    }
    if (this.agents.length > count) {
      this.agents.length = count;
    }
    if (this.nextAgent.length !== this.agents.length) {
      this.nextAgent = new Int32Array(this.agents.length);
    }
    this.rebuildSpatialIndex();
  }

  resetPositions(): void {
    for (const agent of this.agents) {
      this.placeAgent(agent);
    }
    this.rebuildSpatialIndex();
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

  getCellOccupancy(index: number): number {
    return this.occupancy[index] ?? 0;
  }

  getMaximumCellOccupancy(): number {
    return this.maximumOccupancy;
  }

  update(
    deltaTime: number,
    mode: NavigationMode,
    goalIndex: number,
    flowField: FlowField,
    speedMultiplier: number,
    useLocalAvoidance = true,
  ): number {
    const goalX = (this.map.getColumn(goalIndex) + 0.5) * this.cellSize;
    const goalY = (this.map.getRow(goalIndex) + 0.5) * this.cellSize;
    let reachedCount = 0;

    // This dynamic grid is intentionally separate from the static cost field. Rebuilding a
    // destination field for every moving body would destroy its reuse advantage; indexing the
    // crowd instead gives local steering an O(n) setup and small neighboring-cell queries.
    this.rebuildSpatialIndex();

    for (let agentIndex = 0; agentIndex < this.agents.length; agentIndex += 1) {
      const agent = this.agents[agentIndex];
      if (agent.reached) {
        reachedCount += 1;
        continue;
      }

      let directionX = 0;
      let directionY = 0;
      if (mode === 'flow') {
        const cellIndex = this.getAgentCellIndex(agent);
        // This O(1) sample is the amortization payoff: planning cost belongs to the field,
        // while each agent only reads the direction stored under its current cell.
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

      if (useLocalAvoidance) {
        const avoidance = this.getAvoidanceVector(agentIndex, agent);
        directionX += avoidance.x;
        directionY += avoidance.y;
        const steeringLength = Math.hypot(directionX, directionY);
        if (steeringLength > 1) {
          directionX /= steeringLength;
          directionY /= steeringLength;
        }
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

  private rebuildSpatialIndex(): void {
    this.occupancy.fill(0);
    this.bucketHeads.fill(-1);
    this.maximumOccupancy = 0;

    for (let index = 0; index < this.agents.length; index += 1) {
      const agent = this.agents[index];
      if (agent.reached) {
        this.nextAgent[index] = -1;
        continue;
      }
      const cellIndex = this.getAgentCellIndex(agent);
      this.nextAgent[index] = this.bucketHeads[cellIndex];
      this.bucketHeads[cellIndex] = index;
      this.occupancy[cellIndex] += 1;
      this.maximumOccupancy = Math.max(this.maximumOccupancy, this.occupancy[cellIndex]);
    }
  }

  private getAvoidanceVector(agentIndex: number, agent: Agent): { x: number; y: number } {
    const cellIndex = this.getAgentCellIndex(agent);
    const column = this.map.getColumn(cellIndex);
    const row = this.map.getRow(cellIndex);
    let separationX = 0;
    let separationY = 0;
    let pressureX = 0;
    let pressureY = 0;

    for (let offsetY = -1; offsetY <= 1; offsetY += 1) {
      for (let offsetX = -1; offsetX <= 1; offsetX += 1) {
        const nextColumn = column + offsetX;
        const nextRow = row + offsetY;
        if (!this.map.inBounds(nextColumn, nextRow)) {
          continue;
        }
        const nextCell = this.map.getIndex(nextColumn, nextRow);
        if (!this.map.isWalkableIndex(nextCell)) {
          continue;
        }

        // Occupancy behaves like a tiny dynamic pressure field layered over global guidance.
        // Empty neighboring cells pull gently; crowded cells push, helping lanes spread before
        // exact body overlap has already occurred.
        const pressureDifference = this.occupancy[cellIndex] - this.occupancy[nextCell];
        pressureX += offsetX * pressureDifference;
        pressureY += offsetY * pressureDifference;

        let neighborIndex = this.bucketHeads[nextCell];
        while (neighborIndex >= 0) {
          if (neighborIndex !== agentIndex) {
            const neighbor = this.agents[neighborIndex];
            let deltaX = agent.x - neighbor.x;
            let deltaY = agent.y - neighbor.y;
            let distance = Math.hypot(deltaX, deltaY);
            const desiredDistance = agent.radius + neighbor.radius + 1.5;
            if (distance < desiredDistance) {
              if (distance < 0.001) {
                // Stable pair-derived directions separate exact spawn overlaps without adding
                // frame-to-frame randomness that would make the crowd shimmer.
                const angle = ((agentIndex * 131 + neighborIndex * 17) % 360) * (Math.PI / 180);
                deltaX = Math.cos(angle);
                deltaY = Math.sin(angle);
                distance = 1;
              }
              const overlap = (desiredDistance - distance) / desiredDistance;
              separationX += (deltaX / distance) * overlap;
              separationY += (deltaY / distance) * overlap;
            }
          }
          neighborIndex = this.nextAgent[neighborIndex];
        }
      }
    }

    const separationLength = Math.hypot(separationX, separationY);
    if (separationLength > 1) {
      separationX /= separationLength;
      separationY /= separationLength;
    }
    const pressureLength = Math.hypot(pressureX, pressureY);
    if (pressureLength > 0) {
      pressureX /= pressureLength;
      pressureY /= pressureLength;
    }

    return {
      x: separationX * 1.15 + pressureX * 0.24,
      y: separationY * 1.15 + pressureY * 0.24,
    };
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
