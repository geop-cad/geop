# Cloudflare Pages Functions

The server side of app.geop-cad.dev: Pages Functions deployed together with
the web app's static build (`../web/dist`), and the budget worker the agent
chat's functions bind. Both are deployed only by GitHub Actions
(`.github/workflows/deploy-app.yml`): first `budget-worker/`, then the Pages
project from this directory (Wrangler discovers `functions/` relative to
the directory it runs in, and reads `wrangler.toml` here, which holds every
binding and var of the project; the secrets are set on the project).

```sh
npm install        # once
npm run typecheck
npm test           # node >= 23.6 (runs the .ts tests directly)
```

- `functions/api/bug-report.ts` — collects bug reports from the editor into
  R2 (see the file for the bindings it needs; they are in `wrangler.toml`).
- `functions/api/agent/` — the hosted CAD assistant: `session.ts`
  (Turnstile, signed cookie), `turn.ts` (one model turn), `status.ts`, and
  `tools.json` (generated: `npm run gen-tools` after
  `cargo build --release -p geop-cad-cli`). Its code is in `lib/agent/`,
  its limits in `lib/agent/config.ts`.
- `budget-worker/` — the worker `geop-agent-budget` with the Durable Object
  `AgentBudget` that enforces the assistant's spend caps, quotas and rates
  (its own `wrangler.toml`, with the caps). The Pages project binds it as
  `AGENT_BUDGET` through `script_name`, so it is deployed first.

See `../docs/agent-chat.md` for the architecture, the contract, the limits,
the secrets and the deploy.
