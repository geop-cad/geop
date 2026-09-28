# Cloudflare Pages Functions

The server side of app.geop-cad.dev: Pages Functions deployed together with
the web app's static build (`../web/dist`). Wrangler discovers them in
`functions/` relative to the directory it runs in, so deploy from here:

```sh
npm install        # once
npm run typecheck
npx wrangler pages deploy ../web/dist --project-name=geop-cad-app
```

- `functions/api/bug-report.ts` — collects bug reports from the editor into
  R2 (see the file for the bindings it needs).
