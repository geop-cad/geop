/** What a request may cost before it is sent, and what it did cost after. All amounts in nanodollars. */
import { COUNT_MARGIN_RATIO, COUNT_MARGIN_TOKENS, type Price } from "./config.ts";

/** The usage fields the Messages API reports, as far as they are billed. */
export interface Usage {
  input_tokens: number;
  output_tokens: number;
  cache_creation_input_tokens?: number | null;
  cache_read_input_tokens?: number | null;
  cache_creation?: { ephemeral_5m_input_tokens?: number | null; ephemeral_1h_input_tokens?: number | null } | null;
}

/**
 * An upper bound on the cost of a request whose prompt `count_tokens`
 * counted as `countedInputTokens`: every input token at the dearest input
 * rate the request can incur (a 5-minute cache write), plus a margin on
 * the count, plus every token `max_tokens` allows at the output rate.
 */
export function reservationFor(price: Price, countedInputTokens: number, maxTokens: number): number {
  const inputBound = Math.ceil(countedInputTokens * (1 + COUNT_MARGIN_RATIO)) + COUNT_MARGIN_TOKENS;
  return inputBound * price.cacheWrite5m + maxTokens * price.output;
}

const count = (n: number | null | undefined) => (typeof n === "number" && Number.isFinite(n) && n > 0 ? n : 0);

/** The cost of a response's `usage`. Cache writes go by TTL when the breakdown is given, as 5-minute writes otherwise. */
export function costOf(price: Price, usage: Usage): number {
  const written = count(usage.cache_creation_input_tokens);
  const written1h = Math.min(written, count(usage.cache_creation?.ephemeral_1h_input_tokens));
  return (
    count(usage.input_tokens) * price.input +
    (written - written1h) * price.cacheWrite5m +
    written1h * price.cacheWrite1h +
    count(usage.cache_read_input_tokens) * price.cacheRead +
    count(usage.output_tokens) * price.output
  );
}
