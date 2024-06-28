# Specifies whether the legacy 4.2 api should be used.
# 1 by default.
# Set to 1 to use 4.2 advertisements.
# Set to 0 to use 5.0 extended advertisements.
BTLE_APP_USE_LEGACY_API?=1

# Run link layer using SDMA.
ENABLE_SDMA?=0

# Copy sdma config from application.
ENABLE_SDMA_UNIFIED_CONFIG?=1

# Firmware version string
# 1.0 in ASCII
FW_VERSION?=0x312E31

# Enable file transfer profile
ENABLE_WDX?=0

