# Test buffer descriptor for testing buffer backend plugin system
# Contains serialized data and hash for verification

uint64 size                    # Number of elements in buffer
string element_type_name       # Type name for validation (e.g., "h" for uint8_t)

# Serialized data (copy of the buffer contents)
uint8[] data

# Hash of the data for verification
# Computed as a simple FNV-1a hash of the data bytes
uint64 data_hash
