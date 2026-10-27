# The editor's UX rules

What every front end (web, VS Code, desktop) follows, and why. Behaviour
belongs in the kernel's editor layer (`geop-core-editor`), so the front ends
agree; a front end only draws what it is sent and sends what the user did.

## Picking

- **Pick in what is seen.** A pick is made in the part as the viewer was
  told to draw it: what the scene hides (`SceneState::hidden`, a placed
  part and everything placed in it, a solid, a sketch, a datum) is neither
  hit nor hides what is behind it (`view::Visible`).
- **Light what a click holds.** While a field waits for a pick, the hover
  lights and draws exactly what a click there would put in the field —
  a mate's face with the frame put on it, a pattern's whole feature — not
  the raw element under the pointer (`selectable::previewed`).
- **Show what is only for picking only while it can be picked.** The
  datums of placed parts (`Drawer::hidden`) are shown while a field that
  takes a datum waits, and hidden otherwise, the whole time a part is
  placed included.

## Keys

- **The kernel names the keys.** An operation's key is its
  `OperationInfo::shortcut` (a letter, `Shift+` one for the less used),
  unique among the operations (`every_shortcut_starts_one_operation`);
  `F` and `Home` are the view's, `?` lists the keys, `Ctrl/⌘+K` opens the
  command palette, which searches operations, steps, examples, tools,
  inspectors, exports and actions and shows each one's key.
- **Plain letters belong to the step being edited** (its tools); the
  operations' keys work while no step is edited. Chords with Ctrl/⌘ work
  always; `Ctrl/⌘+Enter` commits from anywhere, a field included.
- **Escape ends what is in progress, Enter finishes it.** With nothing in
  progress (no field picking, nothing selected or dragged, no tool in
  hand), Enter commits the step and Escape cancels it — Escape only before
  the step was changed, so a key never throws work away
  (`StepEditor::at_rest`). A commit by key is one edit to undo. The
  kernel decides this, so the web app, VS Code and the desktop app agree;
  `Ctrl/⌘+Backspace` cancels a step whatever is in progress, in both apps.
- Every tooltip and menu entry of something with a key says the key, as
  the platform writes it (`shortcuts.ts`: `⇧F` on a Mac, `Shift+F`
  elsewhere).

## Responsiveness

- **Count the work of a pointer event, not its milliseconds.** A hover
  draws no part (`view::parts_drawn`), builds the step's form at most three
  times (`editable::forms_built`), and solves nothing: what a step pulls
  was solved for at the edit that pulled it.
- **One pointer event in flight.** The viewer sends the latest hover or
  drag once the last is answered; it never queues them.
- **The pointer's moves are not "busy".** Only commands the user waits for
  (open, commit, load) show the spinner.
- **Nothing per frame that only changes per update.** The render loop
  scales what is drawn at a constant size on screen; which datums are
  drawn and lit, and the visuals, are rebuilt only when the update that
  decides them arrives (compared by identity, not by serializing them each
  frame). An update equal to the last one shown is kept as it was, so
  nothing that draws it renders again.
- Send each thing once: a placed part's component and a face's mesh are
  sent once, by key (`editor::kept`).

## Layout and words

- A placed part is listed by its name, the file it is from said beside it,
  quieter; a step by its id, its tooltip saying what it is.
- An empty panel says what to do next, naming the key.
- A refusal names what to do: "pick the face first", not "invalid".

## Checking it

- An editor test per behaviour, driving the editor as a front end does
  (`geop_core_editor::testing`), in the crate that offers it.
- What only a browser shows — the palette, the keys, the spinner — is
  checked by `npm run e2e`; front-end logic without a browser
  (`shortcuts.ts`) by `npm test`.
