import { AStarPlanner } from './algorithms/aStar';
import { FlowField } from './algorithms/flowField';
import { Crowd, type NavigationMode } from './simulation/crowd';
import {
  GridMap,
  TERRAIN_NORMAL,
  TERRAIN_ROUGH,
  TERRAIN_WALL,
  type PaintTool,
  type TerrainPreset,
} from './simulation/gridMap';

interface NavigationStats {
  duration: number;
  expandedNodes: number;
  routes: number;
  unreachable: number;
}

interface ComparisonResult {
  flowDuration: number;
  flowNodes: number;
  aStarDuration: number;
  aStarNodes: number;
  routes: number;
  destinationGroups: number;
  maxCostDelta: number;
  reachabilityMismatches: number;
  scaling: ComparisonScalingPoint[];
}

interface ComparisonScalingPoint {
  routes: number;
  flowDuration: number;
  aStarDuration: number;
}

interface AppElements {
  themeButton: HTMLButtonElement;
  notesButton: HTMLButtonElement;
  closeDialogButton: HTMLButtonElement;
  dialog: HTMLDialogElement;
  resetButton: HTMLButtonElement;
  stressButton: HTMLButtonElement;
  compareButton: HTMLButtonElement;
  experimentButtons: NodeListOf<HTMLButtonElement>;
  experimentSummary: HTMLElement;
  pauseToggle: HTMLInputElement;
  agentSlider: HTMLInputElement;
  agentValue: HTMLElement;
  speedSlider: HTMLInputElement;
  speedValue: HTMLElement;
  brushSlider: HTMLInputElement;
  brushValue: HTMLElement;
  presetSelect: HTMLSelectElement;
  destinationSlider: HTMLInputElement;
  destinationValue: HTMLElement;
  vectorsToggle: HTMLInputElement;
  heatmapToggle: HTMLInputElement;
  routesToggle: HTMLInputElement;
  densityToggle: HTMLInputElement;
  avoidanceToggle: HTMLInputElement;
  modeDescription: HTMLElement;
  modeButtons: NodeListOf<HTMLButtonElement>;
  toolButtons: NodeListOf<HTMLButtonElement>;
  canvas: HTMLCanvasElement;
  contextLabel: HTMLElement;
  insightTitle: HTMLElement;
  insightBody: HTMLElement;
  insightReuse: HTMLElement;
  insightBuilds: HTMLElement;
  insightWins: HTMLElement;
  insightLoses: HTMLElement;
  insightRead: HTMLElement;
  fpsBadge: HTMLElement;
  frameTime: HTMLElement;
  updateTime: HTMLElement;
  renderTime: HTMLElement;
  buildTime: HTMLElement;
  expandedNodes: HTMLElement;
  routeCount: HTMLElement;
  reachedCount: HTMLElement;
  totalCount: HTMLElement;
  unreachableCount: HTMLElement;
  peakOccupancy: HTMLElement;
  comparisonStatus: HTMLElement;
  comparisonFlowTime: HTMLElement;
  comparisonFlowNodes: HTMLElement;
  comparisonAStarTime: HTMLElement;
  comparisonAStarNodes: HTMLElement;
  comparisonRatio: HTMLElement;
  comparisonRoutes: HTMLElement;
  comparisonAudit: HTMLElement;
  comparisonScaling: HTMLElement;
}

type ThemeName = 'paper' | 'midnight';
type ExperimentName = 'shared-army' | 'small-squad' | 'split-orders' | 'weighted-costs';

interface ExperimentPreset {
  agents: number;
  destinations: number;
  terrain: TerrainPreset;
  mode: NavigationMode;
  summary: string;
}

const COLUMNS = 64;
const ROWS = 40;
const CELL_SIZE = 15;
const CANVAS_WIDTH = COLUMNS * CELL_SIZE;
const CANVAS_HEIGHT = ROWS * CELL_SIZE;
const DEFAULT_AGENT_COUNT = 1200;
const MAX_AGENT_COUNT = 5000;
const COMPARISON_ROUTE_LIMIT = 3000;
const MAX_DESTINATION_GROUPS = 32;
const DEFAULT_PRESET: TerrainPreset = 'chokepoints';

const EXPERIMENTS: Record<ExperimentName, ExperimentPreset> = {
  'shared-army': {
    agents: 5_000,
    destinations: 1,
    terrain: 'chokepoints',
    mode: 'flow',
    summary: 'Five thousand agents reuse one destination field: the classic flow-field workload.',
  },
  'small-squad': {
    agents: 100,
    destinations: 1,
    terrain: 'maze',
    mode: 'astar',
    summary: 'Only one hundred routes are requested, so shared preprocessing has less work to amortize.',
  },
  'split-orders': {
    agents: 2_000,
    destinations: 16,
    terrain: 'islands',
    mode: 'flow',
    summary: 'Sixteen destination groups require sixteen fields and reduce reuse to 125 agents per field.',
  },
  'weighted-costs': {
    agents: 1_200,
    destinations: 1,
    terrain: 'islands',
    mode: 'flow',
    summary: 'Rough terrain tests whether shared integration costs agree with independent A* route costs.',
  },
};

/**
 * Coordinates the editable world, both path planners, crowd movement, renderer, controls, and benchmark telemetry.
 *
 * The comparison intentionally measures path construction separately from movement. That keeps the central question
 * clear: how much planning work is repeated when many agents share one destination?
 */
export class FlowFieldCaseStudyApp {
  private readonly root: HTMLDivElement;
  private readonly map = new GridMap(COLUMNS, ROWS);
  private readonly flowField = new FlowField(this.map);
  private readonly aStar = new AStarPlanner(this.map);
  private readonly crowd = new Crowd(this.map, CELL_SIZE);

  private elements!: AppElements;
  private context!: CanvasRenderingContext2D;
  private mode: NavigationMode = 'flow';
  private tool: PaintTool = 'goal';
  private theme: ThemeName = 'midnight';
  private preset: TerrainPreset = DEFAULT_PRESET;
  private goalIndex = this.map.getIndex(56, 20);
  private brushSize = 1;
  private speedMultiplier = 1;
  private showVectors = true;
  private showHeatmap = true;
  private showRoutes = true;
  private showDensity = true;
  private useLocalAvoidance = true;
  private destinationGroupCount = 1;
  private isPaused = false;
  private isPainting = false;
  private hoverColumn = -1;
  private hoverRow = -1;
  private lastFrameStart = performance.now();
  private fps = 60;
  private fpsFrames = 0;
  private fpsTime = 0;
  private frameInterval = 1000 / 60;
  private updateDuration = 0;
  private renderDuration = 0;
  private reachedCount = 0;
  private maxIntegrationCost = 1;
  private navigationStats: NavigationStats = {
    duration: 0,
    expandedNodes: 0,
    routes: 1,
    unreachable: 0,
  };
  private comparison: ComparisonResult | null = null;
  private countDebounceId: number | null = null;
  private rebuildDebounceId: number | null = null;

  constructor(root: HTMLDivElement) {
    this.root = root;
  }

  mount(): void {
    this.root.innerHTML = this.renderMarkup();
    this.elements = this.captureElements();

    const context = this.elements.canvas.getContext('2d');
    if (!context) {
      throw new Error('Canvas 2D is not supported in this browser.');
    }
    this.context = context;

    this.theme = this.getPreferredTheme();
    this.applyTheme();
    this.map.loadPreset(this.preset);
    this.goalIndex = this.map.findNearestWalkable(56, 20);
    this.crowd.setCount(DEFAULT_AGENT_COUNT);
    this.rebuildNavigation();
    this.bindEvents();
    this.syncControls();
    requestAnimationFrame((timestamp) => this.loop(timestamp));
  }

  private bindEvents(): void {
    this.elements.themeButton.addEventListener('click', () => {
      this.theme = this.theme === 'paper' ? 'midnight' : 'paper';
      this.applyTheme();
    });

    this.elements.notesButton.addEventListener('click', () => this.elements.dialog.showModal());
    this.elements.closeDialogButton.addEventListener('click', () => this.elements.dialog.close());
    this.elements.dialog.addEventListener('click', (event) => {
      const bounds = this.elements.dialog.getBoundingClientRect();
      if (
        event.clientX < bounds.left ||
        event.clientX > bounds.right ||
        event.clientY < bounds.top ||
        event.clientY > bounds.bottom
      ) {
        this.elements.dialog.close();
      }
    });

    this.elements.resetButton.addEventListener('click', () => this.resetCaseStudy());
    this.elements.stressButton.addEventListener('click', () => {
      const nextCount = Math.min(this.crowd.agents.length + 1000, MAX_AGENT_COUNT);
      this.setAgentCount(nextCount);
    });
    this.elements.compareButton.addEventListener('click', () => void this.runComparison());

    this.elements.experimentButtons.forEach((button) => {
      button.addEventListener('click', () => {
        const experiment = button.dataset.experiment;
        if (
          experiment === 'shared-army' ||
          experiment === 'small-squad' ||
          experiment === 'split-orders' ||
          experiment === 'weighted-costs'
        ) {
          this.applyExperiment(experiment);
        }
      });
    });

    this.elements.modeButtons.forEach((button) => {
      button.addEventListener('click', () => {
        const mode = button.dataset.mode;
        if (mode === 'flow' || mode === 'astar') {
          this.mode = mode;
          this.rebuildNavigation();
          this.syncControls();
        }
      });
    });

    this.elements.toolButtons.forEach((button) => {
      button.addEventListener('click', () => {
        const tool = button.dataset.tool;
        if (tool === 'goal' || tool === 'wall' || tool === 'rough' || tool === 'erase') {
          this.tool = tool;
          this.syncControls();
        }
      });
    });

    this.elements.agentSlider.addEventListener('input', () => {
      const count = Number.parseInt(this.elements.agentSlider.value, 10);
      this.elements.agentValue.textContent = count.toLocaleString();
      if (this.countDebounceId !== null) {
        window.clearTimeout(this.countDebounceId);
      }
      this.countDebounceId = window.setTimeout(() => this.setAgentCount(count), 140);
    });

    this.elements.speedSlider.addEventListener('input', () => {
      this.speedMultiplier = Number.parseFloat(this.elements.speedSlider.value);
      this.elements.speedValue.textContent = `${this.speedMultiplier.toFixed(1)}x`;
    });

    this.elements.brushSlider.addEventListener('input', () => {
      this.brushSize = Number.parseInt(this.elements.brushSlider.value, 10);
      this.elements.brushValue.textContent = String(this.brushSize);
    });

    this.elements.presetSelect.addEventListener('change', () => {
      const preset = this.elements.presetSelect.value;
      if (preset === 'open' || preset === 'chokepoints' || preset === 'maze' || preset === 'islands') {
        this.preset = preset;
        this.map.loadPreset(preset);
        this.goalIndex = this.map.findNearestWalkable(
          this.map.getColumn(this.goalIndex),
          this.map.getRow(this.goalIndex),
        );
        this.crowd.resetPositions();
        this.rebuildNavigation();
        this.syncControls();
      }
    });

    this.elements.destinationSlider.addEventListener('input', () => {
      this.destinationGroupCount = Number.parseInt(this.elements.destinationSlider.value, 10);
      this.elements.destinationValue.textContent = String(this.destinationGroupCount);
      this.comparison = null;
      this.updateComparisonTelemetry();
      this.syncExperimentButtons();
      this.updateInsightTelemetry();
    });

    this.elements.vectorsToggle.addEventListener('change', () => {
      this.showVectors = this.elements.vectorsToggle.checked;
    });
    this.elements.heatmapToggle.addEventListener('change', () => {
      this.showHeatmap = this.elements.heatmapToggle.checked;
    });
    this.elements.routesToggle.addEventListener('change', () => {
      this.showRoutes = this.elements.routesToggle.checked;
    });
    this.elements.densityToggle.addEventListener('change', () => {
      this.showDensity = this.elements.densityToggle.checked;
    });
    this.elements.avoidanceToggle.addEventListener('change', () => {
      this.useLocalAvoidance = this.elements.avoidanceToggle.checked;
    });
    this.elements.pauseToggle.addEventListener('change', () => {
      this.isPaused = this.elements.pauseToggle.checked;
    });

    this.elements.canvas.addEventListener('pointerdown', (event) => {
      const cell = this.getPointerCell(event);
      if (!cell) {
        return;
      }

      this.isPainting = true;
      this.elements.canvas.setPointerCapture(event.pointerId);
      if (event.button === 2 || this.tool === 'goal') {
        this.setGoal(cell.column, cell.row);
      } else {
        this.paintTerrain(cell.column, cell.row);
      }
    });

    this.elements.canvas.addEventListener('pointermove', (event) => {
      const cell = this.getPointerCell(event);
      if (!cell) {
        this.hoverColumn = -1;
        this.hoverRow = -1;
        return;
      }

      this.hoverColumn = cell.column;
      this.hoverRow = cell.row;
      this.updateContextLabel();
      if (this.isPainting && this.tool !== 'goal') {
        this.paintTerrain(cell.column, cell.row);
      }
    });

    this.elements.canvas.addEventListener('pointerleave', () => {
      this.hoverColumn = -1;
      this.hoverRow = -1;
      this.updateContextLabel();
    });
    this.elements.canvas.addEventListener('pointerup', () => {
      this.isPainting = false;
    });
    this.elements.canvas.addEventListener('contextmenu', (event) => event.preventDefault());

    document.addEventListener('keydown', (event) => {
      const target = event.target;
      if (target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target instanceof HTMLButtonElement) {
        return;
      }

      if (event.key === ' ') {
        event.preventDefault();
        this.isPaused = !this.isPaused;
        this.elements.pauseToggle.checked = this.isPaused;
      }
      if (event.key.toLowerCase() === 'f') {
        this.mode = 'flow';
        this.rebuildNavigation();
        this.syncControls();
      }
      if (event.key.toLowerCase() === 'a') {
        this.mode = 'astar';
        this.rebuildNavigation();
        this.syncControls();
      }
    });
  }

  private applyExperiment(experiment: ExperimentName): void {
    const preset = EXPERIMENTS[experiment];
    this.mode = preset.mode;
    this.preset = preset.terrain;
    this.destinationGroupCount = preset.destinations;
    this.map.loadPreset(this.preset);
    this.goalIndex = this.map.findNearestWalkable(56, 20);
    this.crowd.setCount(preset.agents);
    this.crowd.resetPositions();
    this.rebuildNavigation();
    this.syncControls();
  }

  private rebuildNavigation(): void {
    this.comparison = null;
    this.goalIndex = this.map.findNearestWalkable(
      this.map.getColumn(this.goalIndex),
      this.map.getRow(this.goalIndex),
    );

    if (this.mode === 'flow') {
      const stats = this.flowField.build(this.goalIndex);
      this.crowd.clearPaths();
      this.navigationStats = {
        duration: stats.duration,
        expandedNodes: stats.expandedNodes,
        routes: 1,
        unreachable: this.map.getWalkableCount() - stats.reachableCells,
      };
      this.refreshMaxIntegrationCost();
    } else {
      const startedAt = performance.now();
      let expandedNodes = 0;
      let unreachable = 0;
      for (const agent of this.crowd.agents) {
        const result = this.aStar.findPath(this.crowd.getAgentCellIndex(agent), this.goalIndex);
        agent.path = result.path;
        agent.pathIndex = result.path.length > 1 ? 1 : 0;
        agent.reached = false;
        expandedNodes += result.expandedNodes;
        if (result.path.length === 0) {
          unreachable += 1;
        }
      }
      const aStarDuration = performance.now() - startedAt;

      // Keep the flow field current so heatmap and vector toggles remain useful while inspecting A* mode.
      const flowStats = this.flowField.build(this.goalIndex);
      this.refreshMaxIntegrationCost();
      this.navigationStats = {
        duration: aStarDuration,
        expandedNodes,
        routes: this.crowd.agents.length,
        unreachable,
      };
      if (flowStats.reachableCells === 0) {
        this.navigationStats.unreachable = this.crowd.agents.length;
      }
    }

    this.reachedCount = 0;
    this.updateNavigationTelemetry();
    this.updateComparisonTelemetry();
    this.syncExperimentButtons();
    this.updateInsightTelemetry();
  }

  private async runComparison(): Promise<void> {
    this.elements.compareButton.disabled = true;
    this.elements.comparisonStatus.textContent =
      'Building destination fields, running actual routes, and auditing path costs...';

    // Yield once so the status change paints before the intentionally synchronous benchmark starts.
    await new Promise<void>((resolve) => window.setTimeout(resolve, 20));

    const routes = Math.min(this.crowd.agents.length, COMPARISON_ROUTE_LIMIT);
    const goals = this.getComparisonGoals(Math.min(this.destinationGroupCount, routes));
    const flowCosts = new Float32Array(routes);
    flowCosts.fill(Number.POSITIVE_INFINITY);
    const flowBuildDurations: number[] = [];
    let flowNodes = 0;

    // A flow field is destination-owned work. Build one real field per requested destination
    // group, then record the cost-to-go seen by every agent assigned to that group.
    for (let group = 0; group < goals.length; group += 1) {
      const flowStats = this.flowField.build(goals[group]);
      flowBuildDurations.push(flowStats.duration);
      flowNodes += flowStats.expandedNodes;
      for (let index = group; index < routes; index += goals.length) {
        const start = this.crowd.getAgentCellIndex(this.crowd.agents[index]);
        flowCosts[index] = this.flowField.integration[start];
      }
    }

    const checkpoints = new Set(
      [1, 10, 100, 500, 1000, routes].filter((count) => count <= routes),
    );
    const scaling: ComparisonScalingPoint[] = [];
    let aStarDuration = 0;
    let aStarNodes = 0;
    let maxCostDelta = 0;
    let reachabilityMismatches = 0;

    for (let index = 0; index < routes; index += 1) {
      const goal = goals[index % goals.length];
      const routeStartedAt = performance.now();
      const result = this.aStar.findPath(
        this.crowd.getAgentCellIndex(this.crowd.agents[index]),
        goal,
      );
      aStarDuration += performance.now() - routeStartedAt;
      aStarNodes += result.expandedNodes;

      const flowReachable = Number.isFinite(flowCosts[index]);
      const aStarReachable = Number.isFinite(result.cost);
      if (flowReachable !== aStarReachable) {
        reachabilityMismatches += 1;
      } else if (flowReachable && aStarReachable) {
        const denominator = Math.max(1, Math.abs(flowCosts[index]), Math.abs(result.cost));
        maxCostDelta = Math.max(
          maxCostDelta,
          (Math.abs(flowCosts[index] - result.cost) / denominator) * 100,
        );
      }

      const routeCount = index + 1;
      if (checkpoints.has(routeCount)) {
        const activeFields = Math.min(goals.length, routeCount);
        const flowDuration = flowBuildDurations
          .slice(0, activeFields)
          .reduce((total, duration) => total + duration, 0);
        scaling.push({
          routes: routeCount,
          flowDuration,
          aStarDuration,
        });
      }
    }

    this.comparison = {
      flowDuration: flowBuildDurations.reduce((total, duration) => total + duration, 0),
      flowNodes,
      aStarDuration,
      aStarNodes,
      routes,
      destinationGroups: goals.length,
      maxCostDelta,
      reachabilityMismatches,
      scaling,
    };

    // The comparison builds several fields into one reusable buffer. Restore the field shown
    // by the live canvas after timing so the benchmark cannot silently change agent guidance.
    this.flowField.build(this.goalIndex);
    this.refreshMaxIntegrationCost();
    this.updateComparisonTelemetry();
    this.updateInsightTelemetry();
    this.elements.compareButton.disabled = false;
  }

  private getComparisonGoals(count: number): number[] {
    const goals: number[] = [this.goalIndex];
    const seen = new Set(goals);

    for (let candidate = 1; goals.length < count && candidate < count * 8; candidate += 1) {
      const column = Math.round(
        this.map.columns * (0.52 + 0.43 * ((candidate * 0.61803398875) % 1)),
      );
      const row = Math.round(
        1 + (this.map.rows - 3) * ((candidate * 0.38196601125) % 1),
      );
      const goal = this.map.findNearestWalkable(column, row);
      if (!seen.has(goal)) {
        seen.add(goal);
        goals.push(goal);
      }
    }

    for (let index = 0; goals.length < count && index < this.map.size; index += 1) {
      if (this.map.isWalkableIndex(index) && !seen.has(index)) {
        seen.add(index);
        goals.push(index);
      }
    }
    return goals;
  }

  private paintTerrain(column: number, row: number): void {
    this.map.applyBrush(column, row, this.brushSize, this.tool);
    this.goalIndex = this.map.findNearestWalkable(
      this.map.getColumn(this.goalIndex),
      this.map.getRow(this.goalIndex),
    );

    if (this.rebuildDebounceId !== null) {
      window.clearTimeout(this.rebuildDebounceId);
    }
    this.rebuildDebounceId = window.setTimeout(() => this.rebuildNavigation(), 80);
  }

  private setGoal(column: number, row: number): void {
    this.goalIndex = this.map.findNearestWalkable(column, row);
    this.rebuildNavigation();
  }

  private setAgentCount(count: number): void {
    this.crowd.setCount(count);
    this.elements.agentSlider.value = String(count);
    this.elements.agentValue.textContent = count.toLocaleString();
    this.elements.stressButton.disabled = count >= MAX_AGENT_COUNT;
    this.rebuildNavigation();
  }

  private resetCaseStudy(): void {
    this.mode = 'flow';
    this.tool = 'goal';
    this.preset = DEFAULT_PRESET;
    this.speedMultiplier = 1;
    this.brushSize = 1;
    this.showVectors = true;
    this.showHeatmap = true;
    this.showRoutes = true;
    this.showDensity = true;
    this.useLocalAvoidance = true;
    this.destinationGroupCount = 1;
    this.isPaused = false;
    this.comparison = null;
    this.map.loadPreset(this.preset);
    this.goalIndex = this.map.findNearestWalkable(56, 20);
    this.crowd.setCount(DEFAULT_AGENT_COUNT);
    this.crowd.resetPositions();
    this.rebuildNavigation();
    this.syncControls();
    this.updateComparisonTelemetry();
  }

  private loop(frameStart: number): void {
    const elapsed = frameStart - this.lastFrameStart;
    this.lastFrameStart = frameStart;
    this.frameInterval = elapsed;
    const deltaTime = Math.min(elapsed / 1000, 0.1);

    if (!this.isPaused) {
      const updateStartedAt = performance.now();
      this.reachedCount = this.crowd.update(
        deltaTime,
        this.mode,
        this.goalIndex,
        this.flowField,
        this.speedMultiplier,
        this.useLocalAvoidance,
      );
      this.updateDuration = performance.now() - updateStartedAt;
    } else {
      this.updateDuration = 0;
    }

    const renderStartedAt = performance.now();
    this.renderCanvas();
    this.renderDuration = performance.now() - renderStartedAt;

    this.fpsFrames += 1;
    this.fpsTime += elapsed;
    if (this.fpsTime >= 300) {
      this.fps = (this.fpsFrames * 1000) / this.fpsTime;
      this.fpsFrames = 0;
      this.fpsTime = 0;
      this.updateFrameTelemetry();
    }

    this.elements.reachedCount.textContent = this.reachedCount.toLocaleString();
    requestAnimationFrame((timestamp) => this.loop(timestamp));
  }

  private renderCanvas(): void {
    const context = this.context;
    const midnight = this.theme === 'midnight';
    context.fillStyle = midnight ? '#09171c' : '#f5f3ed';
    context.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

    if (this.showHeatmap) {
      for (let index = 0; index < this.map.size; index += 1) {
        const cost = this.flowField.integration[index];
        if (!Number.isFinite(cost) || this.map.costs[index] === TERRAIN_WALL) {
          continue;
        }

        const intensity = Math.min(cost / this.maxIntegrationCost, 1);
        const column = this.map.getColumn(index);
        const row = this.map.getRow(index);
        const alpha = 0.04 + intensity * 0.19;
        context.fillStyle = midnight
          ? `rgba(56, 189, 179, ${alpha})`
          : `rgba(20, 127, 133, ${alpha * 0.72})`;
        context.fillRect(column * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE);
      }
    }

    for (let index = 0; index < this.map.size; index += 1) {
      const terrain = this.map.costs[index];
      if (terrain === TERRAIN_NORMAL) {
        continue;
      }

      const x = this.map.getColumn(index) * CELL_SIZE;
      const y = this.map.getRow(index) * CELL_SIZE;
      if (terrain === TERRAIN_WALL) {
        context.fillStyle = midnight ? '#28363c' : '#506068';
        context.fillRect(x + 1, y + 1, CELL_SIZE - 2, CELL_SIZE - 2);
        context.fillStyle = midnight ? '#34464d' : '#62737b';
        context.fillRect(x + 3, y + 3, CELL_SIZE - 6, 3);
      } else if (terrain === TERRAIN_ROUGH) {
        context.fillStyle = midnight ? 'rgba(240, 143, 97, 0.24)' : 'rgba(184, 75, 33, 0.2)';
        context.fillRect(x, y, CELL_SIZE, CELL_SIZE);
      }
    }

    if (this.showDensity) {
      const maximumDensity = Math.max(1, this.crowd.getMaximumCellOccupancy());
      for (let index = 0; index < this.map.size; index += 1) {
        const occupancy = this.crowd.getCellOccupancy(index);
        if (occupancy === 0) {
          continue;
        }
        const intensity = Math.sqrt(occupancy / maximumDensity);
        const x = this.map.getColumn(index) * CELL_SIZE;
        const y = this.map.getRow(index) * CELL_SIZE;
        context.fillStyle = midnight
          ? `rgba(240, 143, 97, ${0.06 + intensity * 0.28})`
          : `rgba(184, 75, 33, ${0.04 + intensity * 0.2})`;
        context.fillRect(x, y, CELL_SIZE, CELL_SIZE);
      }
    }

    context.strokeStyle = midnight ? 'rgba(137, 169, 172, 0.1)' : 'rgba(23, 34, 38, 0.1)';
    context.lineWidth = 1;
    context.beginPath();
    for (let column = 0; column <= COLUMNS; column += 1) {
      context.moveTo(column * CELL_SIZE + 0.5, 0);
      context.lineTo(column * CELL_SIZE + 0.5, CANVAS_HEIGHT);
    }
    for (let row = 0; row <= ROWS; row += 1) {
      context.moveTo(0, row * CELL_SIZE + 0.5);
      context.lineTo(CANVAS_WIDTH, row * CELL_SIZE + 0.5);
    }
    context.stroke();

    if (this.showVectors) {
      context.strokeStyle = midnight ? 'rgba(115, 209, 197, 0.65)' : 'rgba(20, 105, 110, 0.58)';
      context.lineWidth = 1;
      for (let row = 1; row < ROWS; row += 2) {
        for (let column = 1; column < COLUMNS; column += 2) {
          const index = this.map.getIndex(column, row);
          if (this.map.costs[index] === TERRAIN_WALL) {
            continue;
          }
          const vectorX = this.flowField.vectors[index * 2];
          const vectorY = this.flowField.vectors[index * 2 + 1];
          const x = (column + 0.5) * CELL_SIZE;
          const y = (row + 0.5) * CELL_SIZE;
          context.beginPath();
          context.moveTo(x - vectorX * 3, y - vectorY * 3);
          context.lineTo(x + vectorX * 5, y + vectorY * 5);
          context.stroke();
        }
      }
    }

    if (this.mode === 'astar' && this.showRoutes) {
      context.strokeStyle = midnight ? 'rgba(240, 143, 97, 0.18)' : 'rgba(184, 75, 33, 0.2)';
      context.lineWidth = 1;
      const visibleRouteCount = Math.min(this.crowd.agents.length, 28);
      for (let agentIndex = 0; agentIndex < visibleRouteCount; agentIndex += 1) {
        const path = this.crowd.agents[agentIndex].path;
        if (path.length < 2) {
          continue;
        }
        context.beginPath();
        for (let pathIndex = 0; pathIndex < path.length; pathIndex += 1) {
          const node = path[pathIndex];
          const x = (this.map.getColumn(node) + 0.5) * CELL_SIZE;
          const y = (this.map.getRow(node) + 0.5) * CELL_SIZE;
          if (pathIndex === 0) {
            context.moveTo(x, y);
          } else {
            context.lineTo(x, y);
          }
        }
        context.stroke();
      }
    }

    const goalX = (this.map.getColumn(this.goalIndex) + 0.5) * CELL_SIZE;
    const goalY = (this.map.getRow(this.goalIndex) + 0.5) * CELL_SIZE;
    context.strokeStyle = midnight ? '#f08f61' : '#b84b21';
    context.fillStyle = midnight ? 'rgba(240, 143, 97, 0.18)' : 'rgba(184, 75, 33, 0.14)';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(goalX, goalY, 10, 0, Math.PI * 2);
    context.fill();
    context.stroke();
    context.beginPath();
    context.arc(goalX, goalY, 3, 0, Math.PI * 2);
    context.fillStyle = midnight ? '#ffd0b8' : '#8f3513';
    context.fill();

    for (const agent of this.crowd.agents) {
      // Completed agents leave the active crowd instead of accumulating as a stack of circles
      // on the destination. Their completion remains visible in the reached counter.
      if (agent.reached) {
        continue;
      }
      if (agent.tint < 0.34) {
        context.fillStyle = midnight ? 'rgba(115, 209, 197, 0.82)' : 'rgba(20, 127, 133, 0.82)';
      } else if (agent.tint < 0.67) {
        context.fillStyle = midnight ? 'rgba(145, 184, 243, 0.82)' : 'rgba(69, 108, 168, 0.82)';
      } else {
        context.fillStyle = midnight ? 'rgba(155, 224, 168, 0.82)' : 'rgba(63, 138, 88, 0.82)';
      }
      context.beginPath();
      const radius = this.crowd.agents.length > 2_500 ? Math.min(agent.radius, 2.2) : agent.radius;
      context.arc(agent.x, agent.y, radius, 0, Math.PI * 2);
      context.fill();
    }

    if (this.hoverColumn >= 0 && this.hoverRow >= 0) {
      context.strokeStyle = midnight ? '#eef7f5' : '#172226';
      context.lineWidth = 1.5;
      context.strokeRect(
        this.hoverColumn * CELL_SIZE + 1,
        this.hoverRow * CELL_SIZE + 1,
        CELL_SIZE - 2,
        CELL_SIZE - 2,
      );
    }
  }

  private refreshMaxIntegrationCost(): void {
    let maximum = 1;
    for (const cost of this.flowField.integration) {
      if (Number.isFinite(cost) && cost > maximum) {
        maximum = cost;
      }
    }
    this.maxIntegrationCost = maximum;
  }

  private updateNavigationTelemetry(): void {
    this.elements.buildTime.textContent = this.navigationStats.duration.toFixed(2);
    this.elements.expandedNodes.textContent = this.navigationStats.expandedNodes.toLocaleString();
    this.elements.routeCount.textContent = this.navigationStats.routes.toLocaleString();
    this.elements.unreachableCount.textContent = this.navigationStats.unreachable.toLocaleString();
    this.elements.totalCount.textContent = this.crowd.agents.length.toLocaleString();
    this.elements.peakOccupancy.textContent =
      this.crowd.getMaximumCellOccupancy().toLocaleString();
  }

  private updateFrameTelemetry(): void {
    this.elements.fpsBadge.textContent = `${Math.round(this.fps)} FPS`;
    this.elements.fpsBadge.className = 'fps-badge';
    if (this.fps < 45) {
      this.elements.fpsBadge.classList.add('fps-badge--bad');
    } else if (this.fps < 55) {
      this.elements.fpsBadge.classList.add('fps-badge--warn');
    }
    this.elements.frameTime.textContent = this.frameInterval.toFixed(1);
    this.elements.updateTime.textContent = this.updateDuration.toFixed(2);
    this.elements.renderTime.textContent = this.renderDuration.toFixed(2);
    this.elements.peakOccupancy.textContent =
      this.crowd.getMaximumCellOccupancy().toLocaleString();
  }

  private updateComparisonTelemetry(): void {
    if (!this.comparison) {
      this.elements.comparisonStatus.textContent = 'Run both planners against the current crowd and terrain.';
      this.elements.comparisonFlowTime.textContent = '—';
      this.elements.comparisonFlowNodes.textContent = '—';
      this.elements.comparisonAStarTime.textContent = '—';
      this.elements.comparisonAStarNodes.textContent = '—';
      this.elements.comparisonRatio.textContent = '—';
      this.elements.comparisonRoutes.textContent = 'No comparison yet';
      this.elements.comparisonAudit.textContent = 'Cost audit pending';
      this.elements.comparisonScaling.innerHTML =
        '<div class="scaling-empty">Run the comparison to reveal how planner cost changes as route count grows.</div>';
      return;
    }

    const ratio = this.comparison.flowDuration > 0
      ? this.comparison.aStarDuration / this.comparison.flowDuration
      : 0;
    this.elements.comparisonStatus.textContent =
      `Measured on this device with ${this.comparison.destinationGroups} destination ` +
      `${this.comparison.destinationGroups === 1 ? 'group' : 'groups'}.`;
    this.elements.comparisonFlowTime.textContent = `${this.comparison.flowDuration.toFixed(2)} ms`;
    this.elements.comparisonFlowNodes.textContent = `${this.comparison.flowNodes.toLocaleString()} nodes`;
    this.elements.comparisonAStarTime.textContent = `${this.comparison.aStarDuration.toFixed(2)} ms`;
    this.elements.comparisonAStarNodes.textContent = `${this.comparison.aStarNodes.toLocaleString()} nodes`;
    this.elements.comparisonRatio.textContent = `${ratio.toFixed(1)}x`;
    this.elements.comparisonRoutes.textContent =
      `${this.comparison.routes.toLocaleString()} actual A* routes · ` +
      `${this.comparison.destinationGroups} real field builds`;
    this.elements.comparisonAudit.textContent = this.comparison.reachabilityMismatches === 0
      ? `${this.comparison.maxCostDelta.toFixed(4)}% max path-cost delta`
      : `${this.comparison.reachabilityMismatches} reachability mismatches`;
    this.renderScalingComparison();
  }

  private renderScalingComparison(): void {
    if (!this.comparison) {
      return;
    }

    const maximum = Math.max(
      0.001,
      ...this.comparison.scaling.flatMap((point) => [point.flowDuration, point.aStarDuration]),
    );
    this.elements.comparisonScaling.innerHTML = this.comparison.scaling.map((point) => {
      const flowWidth = Math.max(1, (point.flowDuration / maximum) * 100);
      const aStarWidth = Math.max(1, (point.aStarDuration / maximum) * 100);
      return `
        <div class="scaling-row">
          <strong>${point.routes.toLocaleString()} routes</strong>
          <div class="scaling-bars">
            <span class="scaling-bar scaling-bar--flow" style="width:${flowWidth}%"><i>Flow ${point.flowDuration.toFixed(2)} ms</i></span>
            <span class="scaling-bar scaling-bar--astar" style="width:${aStarWidth}%"><i>A* ${point.aStarDuration.toFixed(2)} ms</i></span>
          </div>
        </div>
      `;
    }).join('');
  }

  private syncControls(): void {
    this.elements.modeButtons.forEach((button) => {
      const active = button.dataset.mode === this.mode;
      button.classList.toggle('is-active', active);
      button.setAttribute('aria-pressed', String(active));
    });
    this.elements.toolButtons.forEach((button) => {
      const active = button.dataset.tool === this.tool;
      button.classList.toggle('is-active', active);
      button.setAttribute('aria-pressed', String(active));
    });

    this.elements.agentSlider.value = String(this.crowd.agents.length);
    this.elements.agentValue.textContent = this.crowd.agents.length.toLocaleString();
    this.elements.speedSlider.value = String(this.speedMultiplier);
    this.elements.speedValue.textContent = `${this.speedMultiplier.toFixed(1)}x`;
    this.elements.brushSlider.value = String(this.brushSize);
    this.elements.brushValue.textContent = String(this.brushSize);
    this.elements.presetSelect.value = this.preset;
    this.elements.destinationSlider.value = String(this.destinationGroupCount);
    this.elements.destinationValue.textContent = String(this.destinationGroupCount);
    this.elements.vectorsToggle.checked = this.showVectors;
    this.elements.heatmapToggle.checked = this.showHeatmap;
    this.elements.routesToggle.checked = this.showRoutes;
    this.elements.densityToggle.checked = this.showDensity;
    this.elements.avoidanceToggle.checked = this.useLocalAvoidance;
    this.elements.pauseToggle.checked = this.isPaused;
    this.elements.stressButton.disabled = this.crowd.agents.length >= MAX_AGENT_COUNT;
    this.elements.modeDescription.textContent = this.mode === 'flow'
      ? 'One destination builds one reusable integration and direction field for the entire crowd.'
      : 'Every agent runs an independent A* search and stores its own waypoint path.';
    this.syncExperimentButtons();
    this.updateInsightTelemetry();
    this.updateContextLabel();
  }

  private syncExperimentButtons(): void {
    let summary =
      'Custom workload: combine crowd size, terrain, and destination reuse to change the planning economics.';
    this.elements.experimentButtons.forEach((button) => {
      const experiment = button.dataset.experiment as ExperimentName | undefined;
      const preset = experiment ? EXPERIMENTS[experiment] : undefined;
      const active = Boolean(
        preset &&
        preset.agents === this.crowd.agents.length &&
        preset.destinations === this.destinationGroupCount &&
        preset.terrain === this.preset &&
        preset.mode === this.mode,
      );
      button.classList.toggle('is-active', active);
      button.setAttribute('aria-pressed', String(active));
      if (active && preset) {
        summary = preset.summary;
      }
    });
    this.elements.experimentSummary.textContent = summary;
  }

  private updateInsightTelemetry(): void {
    const agents = this.crowd.agents.length;
    const activeGroups = Math.min(this.destinationGroupCount, agents);
    const reuse = agents / activeGroups;
    this.elements.insightReuse.textContent = `${Math.round(reuse).toLocaleString()} : 1`;
    this.elements.insightBuilds.textContent = String(activeGroups);

    if (this.mode === 'flow') {
      this.elements.insightTitle.textContent = reuse >= 250
        ? 'Shared planning is being amortized across a large crowd.'
        : 'Destination groups are consuming the field-reuse advantage.';
      this.elements.insightBody.textContent =
        `${agents.toLocaleString()} agents are divided across ${activeGroups} destination ` +
        `${activeGroups === 1 ? 'group' : 'groups'}: about ${Math.round(reuse).toLocaleString()} agents reuse each field.`;
    } else {
      this.elements.insightTitle.textContent = reuse >= 250
        ? 'Independent A* is repeating similar destination work for every agent.'
        : 'This smaller reuse group is closer to A*’s natural one-query shape.';
      this.elements.insightBody.textContent =
        `The live A* mode owns ${agents.toLocaleString()} private routes. The controlled comparison also tests ` +
        `${activeGroups} destination ${activeGroups === 1 ? 'group' : 'groups'}.`;
    }

    this.elements.insightWins.textContent =
      'Many agents share each destination and can sample one stable cost field.';
    this.elements.insightLoses.textContent =
      'Few queries reuse a field, or many distinct destinations require separate fields.';
    if (this.comparison) {
      const ratio = this.comparison.flowDuration > 0
        ? this.comparison.aStarDuration / this.comparison.flowDuration
        : 0;
      this.elements.insightRead.textContent =
        `Measured here: ${this.comparison.destinationGroups} field builds versus ` +
        `${this.comparison.routes.toLocaleString()} routes, A*/flow ${ratio.toFixed(1)}x, ` +
        `${this.comparison.maxCostDelta.toFixed(4)}% max cost delta.`;
    } else {
      this.elements.insightRead.textContent =
        'The canvas combines a static goal field with dynamic local separation. Destination groups apply to the controlled comparison below.';
    }
  }

  private updateContextLabel(): void {
    if (this.hoverColumn < 0 || this.hoverRow < 0) {
      this.elements.contextLabel.textContent = this.tool === 'goal'
        ? 'Click to move the shared destination'
        : `Drag to paint ${this.tool} terrain`;
      return;
    }

    const index = this.map.getIndex(this.hoverColumn, this.hoverRow);
    const terrain = this.map.costs[index] === TERRAIN_WALL
      ? 'wall'
      : this.map.costs[index] === TERRAIN_ROUGH
        ? 'rough'
        : 'normal';
    const integration = this.flowField.integration[index];
    const vectorX = this.flowField.vectors[index * 2];
    const vectorY = this.flowField.vectors[index * 2 + 1];
    const costLabel = Number.isFinite(integration) ? integration.toFixed(1) : 'unreachable';
    const directionLabel = vectorX === 0 && vectorY === 0
      ? 'none'
      : `${vectorX.toFixed(2)}, ${vectorY.toFixed(2)}`;
    this.elements.contextLabel.textContent =
      `Cell ${this.hoverColumn},${this.hoverRow} · ${terrain} · cost-to-go ${costLabel} · vector ${directionLabel}`;
  }

  private getPointerCell(event: PointerEvent): { column: number; row: number } | null {
    const bounds = this.elements.canvas.getBoundingClientRect();
    const column = Math.floor(((event.clientX - bounds.left) / bounds.width) * COLUMNS);
    const row = Math.floor(((event.clientY - bounds.top) / bounds.height) * ROWS);
    if (!this.map.inBounds(column, row)) {
      return null;
    }
    return { column, row };
  }

  private getPreferredTheme(): ThemeName {
    const stored = window.localStorage.getItem('flow-field-case-study-theme');
    if (stored === 'paper' || stored === 'midnight') {
      return stored;
    }
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'midnight' : 'paper';
  }

  private applyTheme(): void {
    document.documentElement.dataset.theme = this.theme;
    window.localStorage.setItem('flow-field-case-study-theme', this.theme);
    if (this.elements) {
      this.elements.themeButton.textContent = this.theme === 'paper' ? 'Paper' : 'Midnight';
    }
  }

  private captureElements(): AppElements {
    return {
      themeButton: this.getElement<HTMLButtonElement>('#theme-toggle'),
      notesButton: this.getElement<HTMLButtonElement>('#study-notes'),
      closeDialogButton: this.getElement<HTMLButtonElement>('#close-dialog'),
      dialog: this.getElement<HTMLDialogElement>('#study-dialog'),
      resetButton: this.getElement<HTMLButtonElement>('#reset-demo'),
      stressButton: this.getElement<HTMLButtonElement>('#stress-demo'),
      compareButton: this.getElement<HTMLButtonElement>('#run-comparison'),
      experimentButtons: this.root.querySelectorAll<HTMLButtonElement>('[data-experiment]'),
      experimentSummary: this.getElement<HTMLElement>('#experiment-summary'),
      pauseToggle: this.getElement<HTMLInputElement>('#pause-sim'),
      agentSlider: this.getElement<HTMLInputElement>('#agent-slider'),
      agentValue: this.getElement<HTMLElement>('#agent-value'),
      speedSlider: this.getElement<HTMLInputElement>('#speed-slider'),
      speedValue: this.getElement<HTMLElement>('#speed-value'),
      brushSlider: this.getElement<HTMLInputElement>('#brush-slider'),
      brushValue: this.getElement<HTMLElement>('#brush-value'),
      presetSelect: this.getElement<HTMLSelectElement>('#preset-select'),
      destinationSlider: this.getElement<HTMLInputElement>('#destination-slider'),
      destinationValue: this.getElement<HTMLElement>('#destination-value'),
      vectorsToggle: this.getElement<HTMLInputElement>('#show-vectors'),
      heatmapToggle: this.getElement<HTMLInputElement>('#show-heatmap'),
      routesToggle: this.getElement<HTMLInputElement>('#show-routes'),
      densityToggle: this.getElement<HTMLInputElement>('#show-density'),
      avoidanceToggle: this.getElement<HTMLInputElement>('#local-avoidance'),
      modeDescription: this.getElement<HTMLElement>('#mode-description'),
      modeButtons: this.root.querySelectorAll<HTMLButtonElement>('[data-mode]'),
      toolButtons: this.root.querySelectorAll<HTMLButtonElement>('[data-tool]'),
      canvas: this.getElement<HTMLCanvasElement>('#navigation-canvas'),
      contextLabel: this.getElement<HTMLElement>('#context-label'),
      insightTitle: this.getElement<HTMLElement>('#insight-title'),
      insightBody: this.getElement<HTMLElement>('#insight-body'),
      insightReuse: this.getElement<HTMLElement>('#insight-reuse'),
      insightBuilds: this.getElement<HTMLElement>('#insight-builds'),
      insightWins: this.getElement<HTMLElement>('#insight-wins'),
      insightLoses: this.getElement<HTMLElement>('#insight-loses'),
      insightRead: this.getElement<HTMLElement>('#insight-read'),
      fpsBadge: this.getElement<HTMLElement>('#fps-badge'),
      frameTime: this.getElement<HTMLElement>('#frame-time'),
      updateTime: this.getElement<HTMLElement>('#update-time'),
      renderTime: this.getElement<HTMLElement>('#render-time'),
      buildTime: this.getElement<HTMLElement>('#build-time'),
      expandedNodes: this.getElement<HTMLElement>('#expanded-nodes'),
      routeCount: this.getElement<HTMLElement>('#route-count'),
      reachedCount: this.getElement<HTMLElement>('#reached-count'),
      totalCount: this.getElement<HTMLElement>('#total-count'),
      unreachableCount: this.getElement<HTMLElement>('#unreachable-count'),
      peakOccupancy: this.getElement<HTMLElement>('#peak-occupancy'),
      comparisonStatus: this.getElement<HTMLElement>('#comparison-status'),
      comparisonFlowTime: this.getElement<HTMLElement>('#comparison-flow-time'),
      comparisonFlowNodes: this.getElement<HTMLElement>('#comparison-flow-nodes'),
      comparisonAStarTime: this.getElement<HTMLElement>('#comparison-astar-time'),
      comparisonAStarNodes: this.getElement<HTMLElement>('#comparison-astar-nodes'),
      comparisonRatio: this.getElement<HTMLElement>('#comparison-ratio'),
      comparisonRoutes: this.getElement<HTMLElement>('#comparison-routes'),
      comparisonAudit: this.getElement<HTMLElement>('#comparison-audit'),
      comparisonScaling: this.getElement<HTMLElement>('#comparison-scaling'),
    };
  }

  private getElement<T extends Element>(selector: string): T {
    const element = this.root.querySelector<T>(selector);
    if (!element) {
      throw new Error(`Expected element ${selector}.`);
    }
    return element;
  }

  private renderMarkup(): string {
    return `
      <main class="shell">
        <header class="hero">
          <div class="hero__copy">
            <div class="brand-mark" aria-hidden="true"><span>→</span><span>↘</span><span>↗</span></div>
            <div>
              <div class="hero__meta">
                <div class="eyebrow">Navigation systems · Case study 002</div>
                <span class="repo-badge">Public project</span>
              </div>
              <h1>Flow Field <span>Pathfinding</span></h1>
              <div class="hero__subtitle">One shared field. Thousands of agents. One destination.</div>
              <p>
                An interactive TypeScript case study comparing a reusable flow field with independent A* searches for
                RTS and simulation crowds that need to move at scale.
              </p>
              <div class="hero__proof" aria-label="Project highlights">
                <span><i class="language-dot language-dot--typescript" aria-hidden="true"></i>TypeScript</span>
                <span><i class="language-dot language-dot--canvas" aria-hidden="true"></i>Canvas 2D</span>
                <span>5k agents</span>
                <span>Weighted terrain</span>
              </div>
            </div>
          </div>
          <div class="hero__actions">
            <a
              class="button button--source"
              href="https://github.com/LilBoyWander/flow-field-pathfinding"
              target="_blank"
              rel="noreferrer"
            ><span class="source-mark" aria-hidden="true">&lt;/&gt;</span>View source</a>
            <button class="button button--theme" id="theme-toggle" type="button">Midnight</button>
            <button class="button" id="study-notes" type="button"><span class="button__spark"></span>Study notes</button>
          </div>
        </header>

        <section class="workspace">
          <section class="stage" aria-labelledby="demo-title">
            <div class="stage__toolbar">
              <div class="stage__meta">
                <div class="stage__index" aria-hidden="true">FF</div>
                <div>
                  <div class="stage__path">demo / shared-destination-routing</div>
                  <h2 id="demo-title">Interactive crowd benchmark</h2>
                  <div class="microcopy">Paint terrain, move the goal, then compare the planning work.</div>
                </div>
              </div>
              <div class="stage__actions">
                <button class="button button--quiet" id="reset-demo" type="button">Reset</button>
                <button class="button button--primary" id="stress-demo" type="button">Stress +1k</button>
              </div>
            </div>

            <div class="mode-switch" aria-label="Navigation algorithm">
              <button class="mode-switch__button is-active" data-mode="flow" type="button" aria-pressed="true">
                <span>Shared flow field</span><small>One field build</small>
              </button>
              <button class="mode-switch__button" data-mode="astar" type="button" aria-pressed="false">
                <span>Independent A*</span><small>One search per agent</small>
              </button>
            </div>
            <p class="mode-description" id="mode-description"></p>

            <section class="experiment-lab" aria-labelledby="experiment-title">
              <div class="experiment-lab__copy">
                <span>Try the decision</span>
                <strong id="experiment-title">Planner experiments</strong>
                <p id="experiment-summary">${EXPERIMENTS['shared-army'].summary}</p>
              </div>
              <div class="experiment-grid">
                <button data-experiment="shared-army" type="button"><b>Shared army</b><small>5K agents · 1 field</small></button>
                <button data-experiment="small-squad" type="button"><b>Small squad</b><small>100 agents · 1 goal</small></button>
                <button data-experiment="split-orders" type="button"><b>Split orders</b><small>2K agents · 16 fields</small></button>
                <button data-experiment="weighted-costs" type="button"><b>Weighted costs</b><small>Audit rough terrain</small></button>
              </div>
            </section>

            <section class="insight" aria-live="polite">
              <div class="insight__main">
                <div class="panel__kicker">Key insight / current planning shape</div>
                <h3 id="insight-title">Shared planning is being amortized across a large crowd.</h3>
                <p id="insight-body"></p>
              </div>
              <div class="insight__signal"><span>Agents per field</span><strong id="insight-reuse">1,200 : 1</strong></div>
              <div class="insight__signal"><span>Fields required</span><strong id="insight-builds">1</strong></div>
              <div class="lesson-strip">
                <div><span>Wins when</span><p id="insight-wins"></p></div>
                <div><span>Loses when</span><p id="insight-loses"></p></div>
                <div class="lesson-strip__read"><span>Read this workload</span><p id="insight-read"></p></div>
              </div>
            </section>

            <div class="canvas-shell">
              <canvas
                id="navigation-canvas"
                width="${CANVAS_WIDTH}"
                height="${CANVAS_HEIGHT}"
                aria-label="Interactive pathfinding grid with moving agents"
              ></canvas>
              <div class="canvas-hud">
                <div><span>Agents</span><strong id="total-count">0</strong></div>
                <i aria-hidden="true"></i>
                <div><span>Reached</span><strong id="reached-count">0</strong></div>
                <i aria-hidden="true"></i>
                <div class="canvas-hud__accent"><span>Build</span><strong><b id="build-time">0.00</b> ms</strong></div>
              </div>
              <div class="canvas-context"><span class="canvas-context__cursor"></span><span id="context-label"></span></div>
            </div>

            <div class="stage-foot">
              <span><b>Interaction</b> click goal · drag terrain · right-click goal</span>
              <span><kbd>F</kbd> flow <kbd>A</kbd> A* <kbd>Space</kbd> pause</span>
            </div>
          </section>

          <aside class="sidebar" aria-label="Navigation controls and telemetry">
            <section class="panel panel--performance">
              <div class="panel__header">
                <div><div class="panel__kicker">Live telemetry</div><h3>Frame health</h3></div>
                <output class="fps-badge" id="fps-badge">60 FPS</output>
              </div>
              <div class="metric metric--wide">
                <div><span>Frame interval</span><small>Actual time between animation frames</small></div>
                <strong><b id="frame-time">0.0</b> ms</strong>
              </div>
              <div class="metric-grid">
                <div class="metric"><span>Movement update</span><strong><b id="update-time">0.00</b> ms</strong></div>
                <div class="metric"><span>Canvas render</span><strong><b id="render-time">0.00</b> ms</strong></div>
              </div>
            </section>

            <section class="panel">
              <div class="panel__header">
                <div><div class="panel__kicker">Workload</div><h3>Simulation</h3></div>
              </div>
              <div class="control-stack">
                <label class="range-row">
                  <span><b>Agent count</b><small>Independent A* stores one route per agent</small></span>
                  <output id="agent-value">1,200</output>
                  <input id="agent-slider" type="range" min="100" max="${MAX_AGENT_COUNT}" step="100" value="${DEFAULT_AGENT_COUNT}" />
                </label>
                <label class="range-row">
                  <span><b>Movement speed</b><small>Planning measurements are unaffected</small></span>
                  <output id="speed-value">1.0x</output>
                  <input id="speed-slider" type="range" min="0.2" max="2.5" step="0.1" value="1" />
                </label>
                <label class="select-row">
                  <span><b>Terrain scenario</b><small>Repeatable comparison workloads</small></span>
                  <select id="preset-select">
                    <option value="open">Open field</option>
                    <option value="chokepoints">Chokepoints</option>
                    <option value="maze">Alternating maze</option>
                    <option value="islands">Weighted islands</option>
                  </select>
                </label>
                <label class="range-row">
                  <span><b>Destination groups</b><small>Controlled comparison only: one flow field per distinct goal</small></span>
                  <output id="destination-value">1</output>
                  <input id="destination-slider" type="range" min="1" max="${MAX_DESTINATION_GROUPS}" step="1" value="1" />
                </label>
              </div>
            </section>

            <section class="panel">
              <div class="panel__header">
                <div><div class="panel__kicker">Edit the world</div><h3>Terrain tools</h3></div>
              </div>
              <div class="tool-grid">
                <button class="tool-button is-active" data-tool="goal" type="button">Goal</button>
                <button class="tool-button" data-tool="wall" type="button">Wall</button>
                <button class="tool-button" data-tool="rough" type="button">Rough cost</button>
                <button class="tool-button" data-tool="erase" type="button">Erase</button>
              </div>
              <label class="range-row range-row--compact">
                <span><b>Brush radius</b></span>
                <output id="brush-value">1</output>
                <input id="brush-slider" type="range" min="1" max="4" step="1" value="1" />
              </label>
            </section>

            <section class="panel">
              <div class="panel__header">
                <div><div class="panel__kicker">Latest rebuild</div><h3>Planner work</h3></div>
              </div>
              <dl class="stats-grid">
                <dt>Nodes expanded</dt><dd id="expanded-nodes">0</dd>
                <dt>Routes built</dt><dd id="route-count">0</dd>
                <dt>Unreachable</dt><dd id="unreachable-count">0</dd>
                <dt>Peak agents in one cell</dt><dd id="peak-occupancy">0</dd>
              </dl>
            </section>
          </aside>
        </section>

        <section class="inspect-bar" aria-labelledby="inspect-title">
          <div class="inspect-bar__header">
            <div class="panel__kicker">Inspect</div>
            <h3 id="inspect-title">Debug and steering layers</h3>
            <p>Toggle observability and local crowd behavior without changing planner-build measurements.</p>
          </div>
          <div class="inspect-controls">
            <label class="inspect-toggle"><span><b>Flow vectors</b><small>Shared direction field</small></span><span class="switch"><input id="show-vectors" type="checkbox" checked /><i></i></span></label>
            <label class="inspect-toggle"><span><b>Integration heatmap</b><small>Cost-to-go surface</small></span><span class="switch"><input id="show-heatmap" type="checkbox" checked /><i></i></span></label>
            <label class="inspect-toggle"><span><b>Crowd density</b><small>Dynamic occupancy</small></span><span class="switch"><input id="show-density" type="checkbox" checked /><i></i></span></label>
            <label class="inspect-toggle"><span><b>Local separation</b><small>Congestion steering</small></span><span class="switch"><input id="local-avoidance" type="checkbox" checked /><i></i></span></label>
            <label class="inspect-toggle"><span><b>A* route sample</b><small>First 28 private paths</small></span><span class="switch"><input id="show-routes" type="checkbox" checked /><i></i></span></label>
            <label class="inspect-toggle"><span><b>Pause movement</b><small>Planning stays active</small></span><span class="switch"><input id="pause-sim" type="checkbox" /><i></i></span></label>
          </div>
        </section>

        <section class="comparison">
          <div class="comparison__intro">
            <div class="eyebrow">Run the comparison</div>
            <h2>Measure repeated work.</h2>
            <p id="comparison-status">Run both planners against the current crowd and terrain.</p>
            <button class="button button--primary" id="run-comparison" type="button">Benchmark both planners</button>
            <small id="comparison-routes">No comparison yet</small>
            <div class="comparison__audit"><span>Correctness / path-cost agreement</span><strong id="comparison-audit">Cost audit pending</strong><small>Flow integration and A* must agree before speed is meaningful.</small></div>
          </div>
          <div class="comparison__results">
            <div class="comparison__cards">
              <article class="result-card result-card--flow">
                <div><span>Shared flow fields</span><b>One build per destination</b></div>
                <strong id="comparison-flow-time">—</strong>
                <small id="comparison-flow-nodes">—</small>
              </article>
              <div class="ratio-card"><span>A* / flow planning cost</span><strong id="comparison-ratio">—</strong></div>
              <article class="result-card result-card--astar">
                <div><span>Independent A*</span><b>One route per agent</b></div>
                <strong id="comparison-astar-time">—</strong>
                <small id="comparison-astar-nodes">—</small>
              </article>
            </div>
            <div class="scaling-view">
              <div class="scaling-view__header"><span>Measured scaling</span><small>Same terrain and destination-group setting</small></div>
              <div id="comparison-scaling"><div class="scaling-empty">Run the comparison to reveal how planner cost changes as route count grows.</div></div>
            </div>
          </div>
        </section>

        <section class="explanation">
          <div class="explanation__intro">
            <div class="eyebrow">Planning pipeline</div>
            <h2>Build once, sample often.</h2>
            <p>Flow fields move the expensive search from every agent into one shared destination-centric data set.</p>
          </div>
          <ol class="pipeline">
            <li><span>01</span><div><b>Integrate</b><p>Expand outward from the goal and record the cheapest known cost for each reachable cell.</p></div></li>
            <li><span>02</span><div><b>Differentiate</b><p>Point each cell toward its lowest-cost neighbor to create the shared vector field.</p></div></li>
            <li><span>03</span><div><b>Sample</b><p>Every agent reads the vector beneath it instead of owning a complete path search.</p></div></li>
          </ol>
        </section>

        <section class="tradeoffs">
          <article><span>Flow fields</span><h3>Large groups, shared goals</h3><p>Excellent when many agents reuse one destination. Rebuild cost is paid again when terrain or the goal changes.</p></article>
          <article><span>Independent A*</span><h3>Individual routes</h3><p>Flexible for unique destinations and one-off queries, but repeated searches duplicate work across a crowd.</p></article>
          <article><span>NavMesh</span><h3>Continuous walkable space</h3><p>A representation choice rather than a direct replacement. A* and flow-like guidance can both operate over navigation regions.</p></article>
        </section>

        <footer class="footer">
          <span>Case study 002 · Shared navigation fields for crowd-scale movement</span>
          <a href="https://github.com/LilBoyWander/flow-field-pathfinding" target="_blank" rel="noreferrer">View the source on GitHub</a>
        </footer>

        <dialog class="dialog" id="study-dialog">
          <div class="dialog__accent"></div>
          <div class="dialog__body">
            <div class="panel__kicker">Study notes</div>
            <h3>What this comparison means</h3>
            <p>
              A flow field does not make every pathfinding problem cheaper. It amortizes planning when many agents can
              share a destination and terrain-cost model.
            </p>
            <ul>
              <li>The flow benchmark builds one integration field and one vector field.</li>
              <li>The A* benchmark performs a complete, real search for every reported route.</li>
              <li>Movement time is measured separately from path-construction time.</li>
              <li>Local separation uses a dynamic occupancy layer and remains outside planner-build timing.</li>
              <li>NavMesh is discussed as a navigation representation, not presented as a dishonest one-to-one rival.</li>
            </ul>
          </div>
          <div class="dialog__actions"><button class="button" id="close-dialog" type="button">Close</button></div>
        </dialog>
      </main>
    `;
  }
}
