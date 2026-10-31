# The editor's UX rules

What every front end (the web app — in the browser, `geop open` and
`geop app` — and VS Code) follows, and why. Behaviour
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
- **The kernel says what is lit.** What a pick, a hover, a list item or an
  answer's row lights is decided once, by the kernel (`view::lit::Lit`),
  and sent as `Update::lit` — a `Lit` per drawn part, only when it
  changed; a list item or a row pointed at is sent as `Command::Light`. A
  front end draws it and classifies nothing, so the web app and VS Code
  light the same faces, edges, sketches and datums.
- **Show what is only for picking only while it can be picked.** The
  datums of placed parts (`Drawer::hidden`) are shown while a field that
  takes a datum waits, and hidden otherwise, the whole time a part is
  placed included.

## Errors

- **An error stands beside its cause.** A step that does not build because
  of one of its arguments says so by `GeopError::concerning(key)` (or
  `.concerning(key)` on a result), `key` the field's: the dialog shows
  why under that field and outlines it (`Field::failure`), and the
  summary under the fields keeps the whole error. Tag where an operation
  reads a reference or checks a value; an error of the geometry as a
  whole (a blend that cannot be rolled) concerns no one field and is
  left untagged.

## Keys

- **The kernel names the keys.** An operation's key is its
  `OperationInfo::shortcut` (a letter, `Shift+` one for the less used),
  unique among the operations (`every_shortcut_starts_one_operation`);
  `F` and `Home` are the view's, `?` lists the keys, `Ctrl/⌘+K` opens the
  command palette, which searches operations, steps, examples, tools,
  inspectors, exports and actions and shows each one's key. What every
  front end offers besides the operations has its key in the kernel too
  (`geop_core_editor::keys::APP_KEYS`, sent as `ProgramState::keys`): the
  palette, the key list, commit, cancel, undo, redo, fit (`F`), reset
  (`Home`) and the projection (`O`).
- **A key with Shift is another key.** `Shift+B` never starts what `B`
  does; a symbol (`?`) is the symbol, whatever Shift it takes to type.
- **The palette ranks alike everywhere**: the label starting with the
  query, then a word of it, then the letters closest together; a group's
  name finds its entries after those; what cannot be done now last.
- **Plain letters belong to the step being edited** (its tools); the
  operations' keys work while no step is edited. Chords with Ctrl/⌘ work
  always; `Ctrl/⌘+Enter` commits from anywhere, a field included.
- **Escape ends what is in progress, Enter finishes it.** With nothing in
  progress (no field picking, nothing selected or dragged, no tool in
  hand), Enter commits the step and Escape cancels it — Escape only before
  the step was changed, so a key never throws work away
  (`StepEditor::at_rest`). A commit by key is one edit to undo. The
  kernel decides this, so the web app and VS Code agree;
  `Ctrl/⌘+Backspace` cancels a step whatever is in progress.
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

## Feel

- **The view moves alike everywhere.** The wheel zooms towards what is
  under the pointer, which stays under it (`gestures.ts` `zoomToward`:
  0.85 of the distance a notch). A
  tumble or a pan let go keeps 0.85 of its momentum per sixtieth of a
  second, whatever the display's rate. A focus move eases in and out over
  0.55 s (smoothstep).
- **Hover and selection look alike everywhere**: lit faces tinted 70 %
  towards `#ffc94a`, lit edges and points drawn over everything in it;
  visuals hovered `#ffe0a0`, selected `#ffd84a`, a point three pixels
  larger.

## Dialogs

- **Type, Enter, Enter.** A dialog opened with nothing to pick and no
  plane to work in puts its first number or text in the focus, selected.
  Enter applies what was typed; Enter in a field that changes nothing is
  the step's Enter, which (nothing in progress) commits.
- **Numbers move without typing.** Up and down arrows step a plain number
  by its field's step (Shift ten steps, Alt a tenth), applied at once;
  dragging a number's label sideways scrubs it, a step per 4 px, the
  newest value sent once the last is answered. A formula is followed,
  never moved. The web app rounds it (`nudge.ts`,
  tested in `web/test/nudge.test.ts`).
- A field keeps what is typed while it has the focus; the kernel's value
  shows once it has not.

## Layout and words

- A placed part is listed by its name, the file it is from said beside it,
  quieter; a step by its id, its tooltip saying what it is.
- An empty panel says what to do next, naming the key. An empty part
  shows how to start (Sketch, then Extrude, with their keys), the keys
  of the palette, the key list and fit, and examples to open, in both
  apps.
- Every tooltip of something with a key says the key from where the
  kernel names it, never a key written into the front end.
- A refusal names what to do: "pick the face first", not "invalid".

## Checking it

- An editor test per behaviour, driving the editor as a front end does
  (`geop_core_editor::testing`), in the crate that offers it.
- What only a browser shows — the palette, the keys, the spinner — is
  checked by `npm run e2e`; front-end logic without a browser
  (`shortcuts.ts`) by `npm test`.
