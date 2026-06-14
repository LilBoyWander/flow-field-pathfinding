import './style.css';
import { FlowFieldCaseStudyApp } from './app';

const appRoot = document.querySelector<HTMLDivElement>('#app');

if (!appRoot) {
  throw new Error('App root not found.');
}

const app = new FlowFieldCaseStudyApp(appRoot);
app.mount();
