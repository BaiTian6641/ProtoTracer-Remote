# Blocked Vendor Downloads

This note records downloads that could not be completed from the current environment even after retries with PowerShell and `curl.exe`.

## ST

Target:

- `https://www.st.com/resource/en/datasheet/lsm6dso.pdf`

Observed result:

- repeated timeout with zero bytes received

Reason this still matters:

- the board BOM uses `LSM6DSOWTR`, and software work should follow the `LSM6DSO` register-family documentation

## Analog Devices

Targets:

- `https://www.analog.com/media/en/technical-documentation/data-sheets/max17055.pdf`
- `https://www.analog.com/media/en/technical-documentation/user-guides/max17055-user-guide.pdf`

Observed result:

- timeout or connection reset from this environment

## Fallbacks kept locally

The following wrapper pages are stored for traceability only:

- `../LSM6DSO_for_LSM6DSOWTR_alldatasheet_wrapper.html`
- `../MAX17055_alldatasheet_wrapper.html`

These files are not actual PDFs and should be replaced with the official documents when a better network path is available.