import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { FlowFieldCaseStudyApp } from '../src/app';

function createCanvasContext(): CanvasRenderingContext2D {
  return new Proxy({} as CanvasRenderingContext2D, {
    get() {
      return vi.fn();
    },
  });
}

describe('flow-field teaching interface', () => {
  beforeEach(() => {
    document.body.innerHTML = '<div id="app"></div>';
    vi.stubGlobal('requestAnimationFrame', vi.fn(() => 1));
    vi.spyOn(HTMLCanvasElement.prototype, 'getContext')
      .mockReturnValue(createCanvasContext());
    vi.spyOn(window, 'matchMedia').mockReturnValue({
      matches: true,
      media: '(prefers-color-scheme: dark)',
      onchange: null,
      addEventListener: vi.fn(),
      removeEventListener: vi.fn(),
      addListener: vi.fn(),
      removeListener: vi.fn(),
      dispatchEvent: vi.fn(),
    });
  });

  afterEach(() => {
    vi.restoreAllMocks();
    vi.unstubAllGlobals();
    window.localStorage.clear();
  });

  it('mounts the planning lesson and applies split-destination experiments', () => {
    const root = document.querySelector<HTMLDivElement>('#app');
    expect(root).not.toBeNull();
    new FlowFieldCaseStudyApp(root!).mount();

    expect(root!.querySelector('#insight-title')?.textContent).toContain('amortized');
    expect(root!.querySelector<HTMLInputElement>('#local-avoidance')?.checked).toBe(true);
    root!.querySelector<HTMLButtonElement>('[data-experiment="split-orders"]')?.click();

    expect(root!.querySelector<HTMLInputElement>('#agent-slider')?.value).toBe('2000');
    expect(root!.querySelector<HTMLInputElement>('#destination-slider')?.value).toBe('16');
    expect(root!.querySelector<HTMLSelectElement>('#preset-select')?.value).toBe('islands');
    expect(root!.querySelector('#insight-reuse')?.textContent).toBe('125 : 1');
    expect(root!.querySelector('#experiment-summary')?.textContent).toContain('Sixteen');
  });

  it('renders measured scaling and a successful path-cost audit', async () => {
    const root = document.querySelector<HTMLDivElement>('#app');
    new FlowFieldCaseStudyApp(root!).mount();
    root!.querySelector<HTMLButtonElement>('[data-experiment="small-squad"]')?.click();
    root!.querySelector<HTMLButtonElement>('#run-comparison')?.click();

    await vi.waitFor(() => {
      expect(root!.querySelector('#comparison-audit')?.textContent).toContain('% max path-cost delta');
    });
    expect(root!.querySelectorAll('.scaling-row').length).toBeGreaterThan(1);
    expect(root!.querySelector('#comparison-routes')?.textContent).toContain('100 actual A* routes');
  });
});
