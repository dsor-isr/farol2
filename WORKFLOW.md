# Git workflow

- `main`: only real-world validated code
- `dev`: latest integrated development code
- `feature/*`, `fix/*`, `experiment/*`: short-lived branches created from `dev`

## Rules
- Do not commit directly to `main`
- Merge feature/fix branches into `dev`
- Merge `dev` into `main` only after field validation
- Delete short-lived branches after merge
- Tag important stable versions