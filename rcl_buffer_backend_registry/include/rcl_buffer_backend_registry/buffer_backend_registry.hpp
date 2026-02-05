// Copyright 2024 NVIDIA Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_
#define RCL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_

#include <map>
#include <memory>
#include <string>

#include "rcl_buffer_backend/buffer_backend.hpp"
#include "rcl_buffer_backend_registry/visibility_control.h"

// Forward declare pluginlib ClassLoader to avoid header dependency
namespace pluginlib
{
template<class T>
class ClassLoader;
}  // namespace pluginlib

namespace rcl_buffer_backend_registry
{

/// Singleton registry for discovering and managing buffer backend plugins.
/// Uses pluginlib for dynamic plugin discovery and loading.
class BufferBackendRegistry
{
public:
  /// Get the singleton instance.
  RCL_BUFFER_BACKEND_REGISTRY_PUBLIC
  static BufferBackendRegistry & get_instance();

  /// Get a registered backend by name (e.g., "cpu", "cuda").
  /// Returns nullptr if backend not found.
  /// Thread-safe after initial load_plugins() call.
  RCL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::shared_ptr<rcl_buffer::BufferBackend> get_backend(const std::string & name);

  /// Manually register a backend (for built-in backends or testing).
  /// Thread-safe.
  RCL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void register_backend(const std::string & name, std::shared_ptr<rcl_buffer::BufferBackend> backend);

  /// Load all available backend plugins via pluginlib.
  /// Called automatically on first get_backend() if not already loaded.
  /// Thread-safe (uses call_once).
  RCL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void load_plugins();

  /// Get names of all registered backends.
  /// Thread-safe after initial load_plugins() call.
  RCL_BUFFER_BACKEND_REGISTRY_PUBLIC
  std::vector<std::string> get_backend_names() const;

  /// Clear all global state including backends and serialization maps.
  /// Called automatically in destructor to prevent plugin cleanup issues.
  RCL_BUFFER_BACKEND_REGISTRY_PUBLIC
  void clear_global_state();

  // Non-copyable, non-movable
  BufferBackendRegistry(const BufferBackendRegistry &) = delete;
  BufferBackendRegistry & operator=(const BufferBackendRegistry &) = delete;
  BufferBackendRegistry(BufferBackendRegistry &&) = delete;
  BufferBackendRegistry & operator=(BufferBackendRegistry &&) = delete;

private:
  BufferBackendRegistry();
  ~BufferBackendRegistry();

  std::map<std::string, std::shared_ptr<rcl_buffer::BufferBackend>> backends_;
  std::unique_ptr<pluginlib::ClassLoader<rcl_buffer::BufferBackend>> loader_;
  bool plugins_loaded_;
};

}  // namespace rcl_buffer_backend_registry

#endif  // RCL_BUFFER_BACKEND_REGISTRY__BUFFER_BACKEND_REGISTRY_HPP_

