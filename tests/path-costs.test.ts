import { describe, expect, it } from 'vitest';
import { AStarPlanner } from '../src/algorithms/aStar';
import { FlowField } from '../src/algorithms/flowField';
import {
  GridMap,
  TERRAIN_ROUGH,
  TERRAIN_WALL,
  type TerrainPreset,
} from '../src/simulation/gridMap';

function expectCostAgreement(map: GridMap, goal: number, starts: number[]): void {
  const flow = new FlowField(map);
  const aStar = new AStarPlanner(map);
  flow.build(goal);

  for (const start of starts) {
    if (!map.isWalkableIndex(start)) {
      continue;
    }
    const route = aStar.findPath(start, goal);
    const flowCost = flow.integration[start];
    expect(Number.isFinite(flowCost)).toBe(Number.isFinite(route.cost));
    if (Number.isFinite(route.cost)) {
      expect(flowCost).toBeCloseTo(route.cost, 3);
      let current = start;
      let tracedCost = 0;
      const visited = new Set<number>();
      while (current !== goal && visited.size <= map.size) {
        expect(visited.has(current)).toBe(false);
        visited.add(current);
        const offsetX = Math.round(flow.vectors[current * 2]);
        const offsetY = Math.round(flow.vectors[current * 2 + 1]);
        expect(offsetX !== 0 || offsetY !== 0).toBe(true);
        const next = map.getIndex(
          map.getColumn(current) + offsetX,
          map.getRow(current) + offsetY,
        );
        tracedCost += map.costs[next] * Math.hypot(offsetX, offsetY);
        current = next;
      }
      expect(current).toBe(goal);
      expect(tracedCost).toBeCloseTo(flowCost, 3);
    }
  }
}

describe('flow-field path-cost contract', () => {
  it('matches A* across every deterministic terrain preset', () => {
    const presets: TerrainPreset[] = ['open', 'chokepoints', 'maze', 'islands'];
    for (const preset of presets) {
      const map = new GridMap(64, 40);
      map.loadPreset(preset);
      const goal = map.findNearestWalkable(56, 20);
      const starts = Array.from({ length: Math.ceil(map.size / 37) }, (_, index) =>
        Math.min(map.size - 1, index * 37));
      expectCostAgreement(map, goal, starts);
    }
  });

  it('routes around expensive cells when a cheaper detour exists', () => {
    const map = new GridMap(7, 5);
    for (let column = 1; column < 6; column += 1) {
      map.setCost(column, 2, TERRAIN_ROUGH);
    }
    const start = map.getIndex(0, 2);
    const goal = map.getIndex(6, 2);
    const aStar = new AStarPlanner(map);
    const flow = new FlowField(map);

    flow.build(goal);
    const route = aStar.findPath(start, goal);

    expect(Array.from(route.path).some((index) => map.getRow(index) !== 2)).toBe(true);
    expect(flow.integration[start]).toBeCloseTo(route.cost, 3);
    expect(Math.abs(flow.vectors[start * 2 + 1])).toBeGreaterThan(0);
  });

  it('agrees on unreachable cells and rejects diagonal corner cutting', () => {
    const map = new GridMap(5, 5);
    const goal = map.getIndex(4, 4);
    map.setCost(1, 0, TERRAIN_WALL);
    map.setCost(0, 1, TERRAIN_WALL);

    const flow = new FlowField(map);
    const aStar = new AStarPlanner(map);
    flow.build(goal);
    const route = aStar.findPath(map.getIndex(0, 0), goal);

    expect(route.path).toHaveLength(0);
    expect(route.cost).toBe(Number.POSITIVE_INFINITY);
    expect(flow.integration[map.getIndex(0, 0)]).toBe(Number.POSITIVE_INFINITY);
  });
});
