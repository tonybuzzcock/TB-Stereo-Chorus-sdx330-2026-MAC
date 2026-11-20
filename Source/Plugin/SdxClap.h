// SdxClap.h
// The three entry functions clap-wrapper links into every format.

#pragma once

bool tbsdx330_entry_init (const char* pluginPath);
void tbsdx330_entry_deinit();
const void* tbsdx330_entry_get_factory (const char* factoryId);
