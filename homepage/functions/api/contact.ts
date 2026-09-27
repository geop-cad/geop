/**
 * Sends the landing page's contact form straight to an inbox via
 * Cloudflare Email Routing's "Send Email" binding — no storage, so there
 * is nothing here to grow toward any quota. Needs, on this Pages project
 * (Settings -> Functions):
 *   - Email Routing set up for geop-cad.dev (a domain, once verified,
 *     needed regardless), with a destination address verified
 *   - A "Send Email" binding named `CONTACT_EMAIL`, restricted (at
 *     bind time, in the dashboard) to that one destination address —
 *     Cloudflare enforces that restriction itself, so this code cannot
 *     be made to email anyone else even if it tried to
 *   - A KV namespace binding named `RATE_LIMIT` (the same namespace bound
 *     to the app's bug-report function — see
 *     web/functions/api/bug-report.ts — bind it to both projects)
 * Missing bindings degrade gracefully: no RATE_LIMIT means no rate
 * limiting (not a 500); no CONTACT_EMAIL means a clear 503 instead of the
 * message vanishing silently.
 */

const MAX_FIELD_LENGTH = { name: 200, email: 320, message: 5000 };
const MAX_MESSAGES_PER_IP_PER_DAY = 5;
const TO_ADDRESS = "tobi.jacob@gmx.net";
const FROM_ADDRESS = "no-reply@geop-cad.dev";

interface Env {
  CONTACT_EMAIL?: SendEmail;
  RATE_LIMIT?: KVNamespace;
}

/** At most `limit` requests per `ip` per UTC day. Fails closed if the KV write itself fails (e.g. its own quota is exhausted). */
async function checkRateLimit(env: Env, ip: string, limit: number): Promise<boolean> {
  if (!env.RATE_LIMIT) return true; // not bound yet — don't block before it's set up
  const key = `contact:${ip}:${new Date().toISOString().slice(0, 10)}`;
  const count = Number((await env.RATE_LIMIT.get(key)) ?? "0");
  if (count >= limit) return false;
  try {
    await env.RATE_LIMIT.put(key, String(count + 1), { expirationTtl: 60 * 60 * 24 });
  } catch {
    return false;
  }
  return true;
}

/** A header value may not contain a line break — that's how header injection works. Strip rather than reject. */
function sanitizeHeaderValue(value: string): string {
  return value.replace(/[\r\n]+/g, " ").trim();
}

export const onRequestPost: PagesFunction<Env> = async ({ request, env }) => {
  const ip = request.headers.get("CF-Connecting-IP") ?? "unknown";
  if (!(await checkRateLimit(env, ip, MAX_MESSAGES_PER_IP_PER_DAY))) {
    return new Response("Too many messages from this address today — try again tomorrow.", { status: 429 });
  }

  let body: unknown;
  try {
    body = await request.json();
  } catch {
    return new Response("Invalid JSON body", { status: 400 });
  }
  if (typeof body !== "object" || body === null) {
    return new Response("Invalid JSON body", { status: 400 });
  }
  const { name, email, message, company } = body as Record<string, unknown>;
  // Honeypot: hidden from real visitors by CSS on the form, so only a bot
  // filling every field it can see in the DOM would set this. Claim
  // success without sending anything — don't tip it off.
  if (typeof company === "string" && company.trim()) {
    return new Response(JSON.stringify({ ok: true }), {
      status: 200,
      headers: { "content-type": "application/json" },
    });
  }
  if (typeof name !== "string" || typeof email !== "string" || typeof message !== "string") {
    return new Response("name, email and message are required", { status: 400 });
  }
  if (!name.trim() || !email.trim() || !message.trim()) {
    return new Response("name, email and message must not be empty", { status: 400 });
  }
  if (
    name.length > MAX_FIELD_LENGTH.name ||
    email.length > MAX_FIELD_LENGTH.email ||
    message.length > MAX_FIELD_LENGTH.message
  ) {
    return new Response("Field too long", { status: 413 });
  }
  if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email)) {
    return new Response("Invalid email address", { status: 400 });
  }

  if (!env.CONTACT_EMAIL) {
    return new Response("The contact form isn't wired up yet — email instead.", { status: 503 });
  }

  const safeName = sanitizeHeaderValue(name);
  const safeEmail = sanitizeHeaderValue(email);
  const subject = sanitizeHeaderValue(`Geop contact form: ${safeName}`);
  const bodyText = message.replace(/\r\n?/g, "\n");

  // Hand-built rather than via a MIME library: one plain-text part, small
  // enough that a library would only add a dependency, not simplicity.
  const raw =
    `From: Geop Contact Form <${FROM_ADDRESS}>\r\n` +
    `To: ${TO_ADDRESS}\r\n` +
    `Reply-To: ${safeName} <${safeEmail}>\r\n` +
    `Subject: ${subject}\r\n` +
    `MIME-Version: 1.0\r\n` +
    `Content-Type: text/plain; charset="utf-8"\r\n` +
    `\r\n` +
    `${bodyText}\r\n`;

  try {
    const { EmailMessage } = await import("cloudflare:email");
    await env.CONTACT_EMAIL.send(new EmailMessage(FROM_ADDRESS, TO_ADDRESS, raw));
  } catch (e) {
    return new Response(`Failed to send: ${e}`, { status: 502 });
  }

  return new Response(JSON.stringify({ ok: true }), {
    status: 200,
    headers: { "content-type": "application/json" },
  });
};
