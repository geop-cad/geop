import type { Kv } from "../lib/agent/ledger.ts";

/** The Durable Object's synchronous KV, in memory, with values copied as storage would. */
export class MemoryKv implements Kv {
  map = new Map<string, string>();
  get<T>(key: string): T | undefined {
    const value = this.map.get(key);
    return value === undefined ? undefined : (JSON.parse(value) as T);
  }
  put<T>(key: string, value: T) {
    this.map.set(key, JSON.stringify(value));
  }
  delete(key: string) {
    return this.map.delete(key);
  }
  *list<T>({ prefix }: { prefix: string }): Iterable<[string, T]> {
    for (const key of [...this.map.keys()].sort()) if (key.startsWith(prefix)) yield [key, this.get<T>(key) as T];
  }
}
