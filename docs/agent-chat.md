# The hosted agent chat

The web app (app.geop-cad.dev) has a free CAD assistant. The browser runs
the agent loop and the tools: it sends the conversation to our endpoint,
gets back the model's next message, runs the tool calls in its wasm kernel
and sends the results with the next request. The endpoint adds what the
client must not choose (our system prompt, our tools, our model, our
`max_tokens`) and accounts for every request against hard caps. It is
useless as a general LLM proxy: it takes only a CAD conversation with our
tools, and the model is told to refuse anything else.

```
browser ──POST /api/agent/session {turnstile}──▶ Pages Function ──siteverify──▶ Turnstile
        ◀── cookie geop_agent (HMAC, 24 h) ─────┘        │
browser ──POST /api/agent/turn {messages}─────▶ Pages Function
                                                 │ 1 admit      ──▶ AgentBudget (Durable Object,
                                                 │ 2 count_tokens ─▶ Anthropic     worker geop-agent-budget)
                                                 │ 3 reserve    ──▶ AgentBudget
                                                 │ 4 messages   ──▶ Anthropic
                                                 │ 5 settle     ──▶ AgentBudget
        ◀── {message, limits} ───────────────────┘
```

Code: `cloudflare-functions/` — `functions/api/agent/{session,turn,status}.ts`
(the endpoints), `functions/api/agent/tools.json` (the tools),
`lib/agent/` (configuration, validation, accounting, sessions, Turnstile,
the system prompt), `budget-worker/` (the Durable Object), `test/` (unit
tests).

## Contract

- `POST /api/agent/session` `{"turnstile": "<token>"}` — verifies the
  Turnstile token (it must succeed, have the action `agent_session` and the
  hostname `app.geop-cad.dev`), then sets the cookie `geop_agent`
  (HttpOnly, Secure, SameSite=Strict, Path=/api/agent): a session id and
  its expiry, HMAC-signed. Answers `200 {"limits": Limits}`.
- `POST /api/agent/turn` `{"messages": MessageParam[]}` — answers
  `200 {"message": {"content", "stop_reason", "usage"}, "limits": Limits}`.
  A `stop_reason` of `"refusal"` is returned like any other; the browser
  shows it and runs none of that turn's tools.
- `GET /api/agent/status` — `{"available": bool}` (cached 30 s), nothing else.
- `Limits` = `{"turns_left_today": number, "turns_per_day": number}`, per
  user (the smaller of the session's and its address's remainder).
- Errors are `{"error": code, "message": text, "retry_after"?: seconds}`
  (also as a `Retry-After` header): `401 session` (no, bad or expired
  cookie, or Turnstile failed: show Turnstile again), `429 rate`,
  `429 quota`, `503 budget` (daily or lifetime budget spent, the kill
  switch, or the endpoint not configured: "the free assistant is paused"),
  `400 invalid`, `502 upstream`.

What a turn may contain (anything else is `400 invalid`): only the key
`messages`; at most 80 messages, user and assistant alternating, first and
last the user; user blocks `text` (at most 24 000 characters: the program sent as context fits; the chat box caps a typed prompt at 4000) and
`tool_result` (text, or one png/jpeg base64 image of at most 300 KB per
request); assistant blocks `text`, `tool_use` of a tool in `tools.json`,
and `thinking` / `redacted_thinking` as the model returned them; every
`tool_use` answered by a `tool_result` in the next message. The body may be
256 KB besides the image's data. The accepted blocks are rebuilt field by
field, so nothing else the client sends reaches the API.

## Limits

All in `cloudflare-functions/lib/agent/config.ts`:

| limit | value |
|---|---|
| turns per session per UTC day | 40 |
| turns per session per minute | 6 |
| turns per IP per UTC day | 80 |
| turns per IP per minute | 12 |
| new sessions per IP per UTC day | 3 |
| turns per minute, everyone | 60 |
| new sessions per UTC day, everyone | 500 |
| session lifetime | 24 h |
| input tokens per request | 100 000 |
| `max_tokens` | 8192 (thinking counts towards it) |

The money caps are vars of the budget worker
(`budget-worker/wrangler.toml`): `AGENT_DAILY_BUDGET_USD = "20"` (per UTC
day) and `AGENT_TOTAL_BUDGET_USD = "100"` (lifetime). A missing or
unreadable value counts as 0 and closes the endpoint.

The model is `AGENT_MODEL` (default `claude-sonnet-5-5`), with adaptive
thinking (summarized) at `AGENT_EFFORT` (default `medium`; `low`, `medium`
or `high`). A model not in the price table `PRICES` refuses to start, since
its spend could not be accounted for. With Sonnet 5.5 a typical turn costs
about $0.01–0.05 (the tools and system prompt, about 3K tokens, are read
from the cache), so $20 is several hundred turns a day; the largest
possible turn reserves about $0.36.

## How the caps are enforced

Every counter lives in one Durable Object (`AgentBudget`, the instance
named `global`), which handles one call at a time; its methods do not
await, so each check-and-update is atomic. KV would not do: it is
eventually consistent and cannot hold a hard cap.

1. **Admit**: refuses if the budget is spent, the session's or the
   address's turns for the day are used up, or a minute rate is reached;
   records the request in the minute windows. Nothing has been asked of
   the API yet.
2. **Count**: `count_tokens` (free) counts the exact request: our tools,
   our system prompt and the messages, image included. Over 100 000 tokens
   is refused.
3. **Reserve**: an upper bound of the turn's cost — the counted input plus
   10 % and 500 tokens, all at the cache-write rate (1.25x input, the
   dearest any input token can be), plus `max_tokens` at the output rate.
   Refused unless it fits both the day's and the lifetime's remainder,
   counting every reservation still open as spent. Counts the turn against
   the session and the address.
4. **Call** the Messages API, with the SDK's retries off (a retry could be
   billed twice against one reservation) and a 5-minute timeout.
5. **Settle** to the cost of the response's `usage` (input, cache writes,
   cache reads, output). If the API answered a 4xx, no model ran and 0 is
   charged; on a 5xx, a timeout or a dropped connection the whole
   reservation is charged. A reservation never settled (the function died)
   is charged in full after 15 minutes.

The ledger keeps, per UTC day, the spend, turns, sessions and how many
reservations were charged in full (`day:YYYY-MM-DD`, never deleted), and
the lifetime spend (`total`, never reset: it survives redeploys and day
rollovers). The rest (per-user counters, minute windows) is dropped at the
day's end.

### What could still exceed a cap

- `count_tokens` is documented as an estimate. A count more than 10 % +
  500 tokens below what is billed would let a turn cost more than its
  reservation; the log line has `over_reservation: true` when that
  happens.
- The price table must match Anthropic's prices for the model. A price
  change, or a model setting priced higher (we use none: no 1-hour cache,
  no fast mode, no data residency), is not seen.
- A 4xx from the API is taken as unbilled.
- Deleting the Durable Object (or its namespace) forgets the lifetime
  spend.
- Spend on the same API key from anywhere else is not counted at all.

So, as a backstop: **give the endpoint its own Anthropic Console workspace
and API key, and set that workspace's monthly spend limit** (Console →
Settings → Workspaces → Limits), e.g. to $100. That limit holds whatever
happens here.

### How to recharge

Once the lifetime spend reaches `AGENT_TOTAL_BUDGET_USD`, the assistant
stays paused (`503 budget`) until the cap is raised: edit
`AGENT_TOTAL_BUDGET_USD` in `cloudflare-functions/budget-worker/wrangler.toml`
(e.g. to `"150"` for $50 more) and push to `main`: the workflow deploys the
budget worker, which reads the cap on every request. The daily cap is
changed the same way. Raise the Console workspace limit too.

To switch the assistant off, set `AGENT_DISABLED = "1"` in
`cloudflare-functions/wrangler.toml` and push.

### Auditing

`GET /ledger` on the budget worker, with `Authorization: Bearer
<LEDGER_TOKEN>`, answers the lifetime spend, every day's record and the
caps (amounts in nanodollars, 1e-9 USD). It answers 404 unless the secret
`LEDGER_TOKEN` is set on the worker (dashboard: Workers →
geop-agent-budget → Settings → Variables and Secrets), and the worker has no public URL (`workers_dev =
false`): turn `workers_dev` on to read it, or read the Durable Object's
storage in the dashboard. The Pages Function logs one line per turn with
counts and costs only, never a message's content.

## Setup

Everything is deployed by GitHub Actions (`.github/workflows/deploy-app.yml`,
on a push to `main`); nobody deploys with Wrangler by hand. All bindings
and vars are checked in:

- `cloudflare-functions/budget-worker/wrangler.toml` — the budget worker
  `geop-agent-budget`: its Durable Object class `AgentBudget` (SQLite
  storage, migration `v1`) and the money caps `AGENT_DAILY_BUDGET_USD`,
  `AGENT_TOTAL_BUDGET_USD`.
- `cloudflare-functions/wrangler.toml` — the Pages project `geop-cad-app`:
  `pages_build_output_dir`, the Durable Object binding `AGENT_BUDGET`
  (`class_name = "AgentBudget"`, `script_name = "geop-agent-budget"`), the
  vars `AGENT_MODEL`, `AGENT_EFFORT`, `AGENT_DISABLED`, and the bug
  reports' R2 bucket `geop-bug-reports`. With this file the project's
  bindings come from it and no longer from the dashboard: a binding added
  in the dashboard is dropped on the next deploy. The workflow refuses to
  deploy while a `fill-in-` placeholder is left in it.

### Secrets

On the Pages project `geop-cad-app` (set by the owner, not in any file):

| name | what |
|---|---|
| `ANTHROPIC_API_KEY` | the key of a dedicated Console workspace (see the backstop above) |
| `SESSION_SECRET` | random, e.g. `openssl rand -base64 32`; changing it ends every session |
| `TURNSTILE_SECRET` | the Turnstile widget's secret key |

Optional on the budget worker: `LEDGER_TOKEN` (for `GET /ledger`).

Never set `TURNSTILE_ALLOW_TEST_HOSTNAMES` or `ANTHROPIC_BASE_URL` in
production: they are for the local smoke test below.

The web app's build gets `VITE_TURNSTILE_SITE_KEY` (the widget's public
site key, in the workflow). The widget is for the hostname
`app.geop-cad.dev`, rendered with the action `agent_session`; another
hostname must be added to `TURNSTILE_HOSTNAMES` in `lib/agent/turnstile.ts`.

### The workflow

1. Build the web app; `npm ci` and `npm run typecheck` in
   `cloudflare-functions/`.
2. Deploy the budget worker (`wrangler deploy` in
   `cloudflare-functions/budget-worker/`). It goes first: the Pages
   binding names it, and a Durable Object binding to a class that does not
   exist yet fails.
3. Deploy the Pages project (`wrangler pages deploy ../web/dist` in
   `cloudflare-functions/`, which reads its `wrangler.toml`).

The API token in the repository secret `CLOUDFLARE_PAGES_TOKEN` needs, on
account `9f1c6a47335dac47c8e580e269b0ae8d`:

- **Account → Cloudflare Pages → Edit** (deploy the project, its bindings
  from `wrangler.toml`);
- **Account → Workers Scripts → Edit** (upload the budget worker, which
  includes creating its Durable Object class and applying its migrations;
  Durable Objects need no permission of their own);
- **Account → Workers R2 Storage → Read** and **Account → Workers KV
  Storage → Read**, if Wrangler is to check the bug report's bindings
  (not needed for the agent chat).

A change to the limits in `lib/agent/config.ts` is used by both the budget
worker and the Pages Functions; the workflow deploys both on every run.

### Tools

`functions/api/agent/tools.json` is generated from the MCP server's tools,
without `save`, `load_program` and `export` (no file system in the browser;
downloads are started by the user):

```sh
cargo build --release -p geop-cad-cli
cd cloudflare-functions && npm run gen-tools
```

Changing the tools or the system prompt changes the cached prefix (one
cache write) and ends conversations in flight whose thinking blocks were
made with the old prefix.

### Local smoke test

No real API calls: a mock answers in place of the API.

```sh
cd cloudflare-functions
node scripts/mock-anthropic.mjs &   # :8788, answers /v1/messages and /v1/messages/count_tokens
(cd budget-worker && npx wrangler dev --port 8790) &
npx wrangler pages dev <some static dir> --port 8789 \
  --binding ANTHROPIC_API_KEY=test SESSION_SECRET=local \
    TURNSTILE_SECRET=1x0000000000000000000000000000000AA \
    TURNSTILE_ALLOW_TEST_HOSTNAMES=1 ANTHROPIC_BASE_URL=http://127.0.0.1:8788
```

Cloudflare's Turnstile test keys (site key `1x00000000000000000000AA`,
secret `1x0000000000000000000000000000000AA`) pass any token, with the
hostname `example.com` and no action, which only
`TURNSTILE_ALLOW_TEST_HOSTNAMES=1` accepts. V8 reserves far more address
space than it uses, so cap `wrangler dev` with a cgroup
(`systemd-run --user --scope -p MemoryMax=3G -- …`), not `prlimit --as`.

## Why there is no refusal fallback

The API can re-run a declined request on a fallback model
(`fallbacks: "default"`, Claude API only). For Sonnet 5.5 it retries only
`cyber` and `frontier_llm` declines, on Sonnet 5, and a decline before any
output may or may not be billed by category. One turn could then cost two
model runs, and the reservation would have to cover both. A CAD session
should not meet those categories, so the endpoint leaves the fallback out
and returns the refusal.
