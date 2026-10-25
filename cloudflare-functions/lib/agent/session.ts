/** The `geop_agent` cookie: a session id and its expiry, signed with HMAC-SHA256 under `SESSION_SECRET`. */
import { SESSION_LIFETIME_SECONDS } from "./config.ts";

export const COOKIE_NAME = "geop_agent";

const encoder = new TextEncoder();

function base64url(bytes: ArrayBuffer): string {
  let binary = "";
  for (const byte of new Uint8Array(bytes)) binary += String.fromCharCode(byte);
  return btoa(binary).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

function fromBase64url(text: string): Uint8Array<ArrayBuffer> | null {
  if (!/^[A-Za-z0-9_-]*$/.test(text)) return null;
  const binary = atob(text.replace(/-/g, "+").replace(/_/g, "/"));
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes;
}

function key(secret: string, usage: "sign" | "verify"): Promise<CryptoKey> {
  return crypto.subtle.importKey("raw", encoder.encode(secret), { name: "HMAC", hash: "SHA-256" }, false, [usage]);
}

/** The cookie's value for a new session `sid`, valid for SESSION_LIFETIME_SECONDS from `now` (ms). */
export async function signSession(secret: string, sid: string, now: number): Promise<string> {
  const payload = `${sid}.${Math.floor(now / 1000) + SESSION_LIFETIME_SECONDS}`;
  const signature = await crypto.subtle.sign("HMAC", await key(secret, "sign"), encoder.encode(payload));
  return `${payload}.${base64url(signature)}`;
}

/** The session id of a cookie value, or null if it is malformed, forged or expired. */
export async function verifySession(secret: string, value: string, now: number): Promise<string | null> {
  const parts = value.split(".");
  if (parts.length !== 3) return null;
  const [sid, expiry, signature] = parts;
  if (!/^[0-9a-f-]{36}$/.test(sid) || !/^\d{1,12}$/.test(expiry)) return null;
  const bytes = fromBase64url(signature);
  if (!bytes) return null;
  const valid = await crypto.subtle.verify("HMAC", await key(secret, "verify"), bytes, encoder.encode(`${sid}.${expiry}`));
  if (!valid || Number(expiry) * 1000 <= now) return null;
  return sid;
}

export function sessionCookie(value: string): string {
  return `${COOKIE_NAME}=${value}; HttpOnly; Secure; SameSite=Strict; Path=/api/agent; Max-Age=${SESSION_LIFETIME_SECONDS}`;
}

export function cookieValue(header: string | null): string | null {
  for (const part of (header ?? "").split(";")) {
    const [name, ...rest] = part.trim().split("=");
    if (name === COOKIE_NAME) return rest.join("=");
  }
  return null;
}
