/**
 * Collects a bug report into R2. Bind an R2 bucket named `BUG_REPORTS` to
 * this Pages project (Settings -> Functions -> R2 bucket bindings) —
 * nothing else needed to start collecting reports.
 *
 * To also get notified per report, add a Cloudflare Email Routing
 * "Send Email" binding named `REPORT_EMAIL` and extend this handler to
 * send through it; reports still land in R2 either way, so that's a
 * pure addition, not a prerequisite.
 */

const MAX_BODY_BYTES = 5 * 1024 * 1024; // 5 MB: comfortably covers a program plus a description.

interface Env {
  BUG_REPORTS: R2Bucket;
}

export const onRequestPost: PagesFunction<Env> = async ({ request, env }) => {
  const contentLength = Number(request.headers.get("content-length") ?? "0");
  if (contentLength > MAX_BODY_BYTES) {
    return new Response("Report too large", { status: 413 });
  }

  let payload: unknown;
  try {
    payload = await request.json();
  } catch {
    return new Response("Invalid JSON body", { status: 400 });
  }
  if (typeof payload !== "object" || payload === null) {
    return new Response("Invalid JSON body", { status: 400 });
  }

  const id = crypto.randomUUID();
  const key = `${new Date().toISOString().replace(/[:.]/g, "-")}-${id}.json`;
  await env.BUG_REPORTS.put(key, JSON.stringify(payload, null, 2), {
    httpMetadata: { contentType: "application/json" },
  });

  return new Response(JSON.stringify({ ok: true, id }), {
    status: 201,
    headers: { "content-type": "application/json" },
  });
};
