$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$dest = Split-Path -Parent $MyInvocation.MyCommand.Path

$items = @(
    @{
        Name = 'TCA9534_ti.pdf'
        Url = 'https://www.ti.com/lit/ds/symlink/tca9534.pdf'
        Note = 'Official TI datasheet for the TCA9534 GPIO expander.'
    },
    @{
        Name = 'LSM6DSO_for_LSM6DSOWTR_st.pdf'
        Url = 'https://www.st.com/resource/en/datasheet/lsm6dso.pdf'
        Note = 'Official ST LSM6DSO datasheet used for the LSM6DSOWTR BOM variant.'
    },
    @{
        Name = 'VCNL4035X01_vishay.pdf'
        Url = 'https://www.vishay.com/docs/84251/vcnl4035x01.pdf'
        Note = 'Official Vishay datasheet for the VCNL4035X01 gesture and proximity sensor.'
    },
    @{
        Name = 'MAX17055_adi.pdf'
        Url = 'https://www.analog.com/media/en/technical-documentation/data-sheets/max17055.pdf'
        Note = 'Official Analog Devices datasheet for the MAX17055 fuel gauge.'
    },
    @{
        Name = 'MAX17055_user_guide_adi.pdf'
        Url = 'https://www.analog.com/media/en/technical-documentation/user-guides/max17055-user-guide.pdf'
        Note = 'Official Analog Devices MAX17055 user guide.'
    },
    @{
        Name = 'SSD1351_solomon_via_crystalfontz.pdf'
        Url = 'https://www.crystalfontz.com/controllers/uploaded/SSD1351%20v1.5%20Crystalfontz%20Branded.pdf'
        Note = 'Crystalfontz mirror of the Solomon Systech SSD1351 controller datasheet.'
    },
    @{
        Name = 'ESP32-C6_TRM_espressif.pdf'
        Url = 'https://documentation.espressif.com/esp32-c6_technical_reference_manual_en.pdf'
        Note = 'Official ESP32-C6 Technical Reference Manual.'
    },
    @{
        Name = 'ESP32-C6_HW_Guidelines_espressif.pdf'
        Url = 'https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/esp-hardware-design-guidelines-en-master-esp32c6.pdf'
        Note = 'Official ESP32-C6 hardware design guidelines.'
    },
    @{
        Name = 'ESP32-C6_Datasheet_espressif.pdf'
        Url = 'https://documentation.espressif.com/esp32-c6_datasheet_en.pdf'
        Note = 'Official ESP32-C6 series datasheet.'
    }
)

foreach ($item in $items)
{
    $out = Join-Path $dest $item.Name

    try
    {
        Invoke-WebRequest -Uri $item.Url -OutFile $out -MaximumRedirection 10
        $size = (Get-Item $out).Length
        Write-Output ('OK   ' + $item.Name + '   ' + $size + ' bytes')
    }
    catch
    {
        if (Test-Path $out)
        {
            Remove-Item $out -Force
        }
        Write-Output ('FAIL ' + $item.Name + ' :: ' + $_.Exception.Message)
    }
}