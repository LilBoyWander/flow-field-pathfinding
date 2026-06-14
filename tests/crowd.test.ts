import { describe, expect, it } from 'vitest';
import { FlowField } from '../src/algorithms/flowField';
import { Crowd } from '../src/simulation/crowd';
import { GridMap } from '../src/simulation/gridMap';

function placeOverlappingPair(crowd: Crowd): void {
  crowd.setCount(2);
  for (const agent of crowd.agents) {
    agent.x = 37.5;
    agent.y = 37.5;
    agent.velocityX = 0;
    agent.velocityY = 0;
    agent.reached = false;
  }
}

function distanceBetweenAgents(crowd: Crowd): number {
  return Math.hypot(
    crowd.agents[0].x - crowd.agents[1].x,
    crowd.agents[0].y - crowd.agents[1].y,
  );
}

describe('crowd local steering', () => {
  it('separates exact overlaps without changing the static flow field', () => {
    const map = new GridMap(9, 5);
    const goal = map.getIndex(8, 2);
    const field = new FlowField(map);
    field.build(goal);
    const integrationBefore = Array.from(field.integration);

    const separated = new Crowd(map, 15);
    placeOverlappingPair(separated);
    separated.update(1 / 30, 'flow', goal, field, 1, true);

    const unseparated = new Crowd(map, 15);
    placeOverlappingPair(unseparated);
    unseparated.update(1 / 30, 'flow', goal, field, 1, false);

    expect(distanceBetweenAgents(separated)).toBeGreaterThan(0);
    expect(distanceBetweenAgents(unseparated)).toBe(0);
    expect(separated.getMaximumCellOccupancy()).toBe(2);
    expect(Array.from(field.integration)).toEqual(integrationBefore);
  });
});
