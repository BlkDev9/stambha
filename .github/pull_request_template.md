## What this changes

<!-- What it does and why, in a few lines. Link the issue it fixes, if any. -->

## How it was tested

<!-- Tests added or run, and on what hardware if it touches hardware. -->

## Checklist

- [ ] New behavior is opt-in (a boot option) unless it fixes a bug
- [ ] Tests cover the change (named as sentences, like the rest of the repo)
- [ ] Disk/installer changes keep the safety rules: free space only, every write read back, nothing trimmed
- [ ] Translator changes produce the same results as before (differential tests against the reference)
- [ ] No serials, MAC addresses, Wi-Fi names or other personal data in logs, tests or screenshots
