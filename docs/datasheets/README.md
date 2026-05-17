# Datasheet Archive

This folder stores local reference material for the ProtoTracer remote firmware work.

## Valid local PDFs

1. `TCA9534_ti.pdf`
   Source: TI official datasheet
2. `VCNL4035X01_vishay.pdf`
   Source: Vishay official datasheet
3. `SSD1351_solomon_via_crystalfontz.pdf`
   Source: Crystalfontz-hosted mirror of the Solomon Systech SSD1351 controller datasheet
4. `ESP32-C6_Datasheet_espressif.pdf`
   Source: Espressif official datasheet
5. `ESP32-C6_TRM_espressif.pdf`
   Source: Espressif technical reference manual
6. `ESP32-C6_HW_Guidelines_espressif.pdf`
   Source: Espressif hardware design guidelines

## Partial local captures

The following files are HTML wrapper pages kept only because the official vendor PDF endpoints were blocked or timed out from this environment:

1. `LSM6DSO_for_LSM6DSOWTR_alldatasheet_wrapper.html`
2. `MAX17055_alldatasheet_wrapper.html`

These wrappers are not substitute datasheets. They are only local breadcrumbs pointing to alternative download pages.

## Official URLs that should replace the wrappers

1. ST LSM6DSO datasheet used for the `LSM6DSOWTR` software interface:
   `https://www.st.com/resource/en/datasheet/lsm6dso.pdf`
2. Analog Devices MAX17055 datasheet:
   `https://www.analog.com/media/en/technical-documentation/data-sheets/max17055.pdf`
3. Analog Devices MAX17055 user guide:
   `https://www.analog.com/media/en/technical-documentation/user-guides/max17055-user-guide.pdf`

## Re-download script

Use this script to refresh the archive later:

```powershell
Set-Location firmware/docs/datasheets
.\download_datasheets.ps1
```

## Notes

1. The BOM part `LSM6DSOWTR` appears to use the `LSM6DSO` software register interface and datasheet family.
2. The SSD1351 PDF came from a mirrored host because the viewer page URL itself returns HTML.
3. The blocked vendor URLs are documented so they can be retried from a different network or workstation later.