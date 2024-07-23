#ifndef FUSE_CORE__TYPES_HPP_
#define FUSE_CORE__TYPES_HPP_

#include <functional>
#include <memory>

#ifdef USE_ROBIN_HOOD
#include <fuse_core/robin_hood.hpp>
template<
  class Key, 
  class T, 
  class Hash = std::hash<Key>, 
  class KeyEqual = std::equal_to<Key>, 
  class Allocator = std::allocator<std::pair<const Key, T>>
> using UnorderedMap = robin_hood::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

template<
  class Key,
  class Hash = std::hash<Key>,
  class KeyEqual = std::equal_to<Key>,
  class Allocator = std::allocator<Key>
> using UnorderedSet = robin_hood::unordered_set<Key, Hash, KeyEqual, Allocator>;

#else
#include <unordered_map>
#include <unordered_set>
template<
  class Key, 
  class T, 
  class Hash = std::hash<Key>, 
  class KeyEqual = std::equal_to<Key>, 
  class Allocator = std::allocator<std::pair<const Key, T>>
> using UnorderedMap = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

template<
  class Key,
  class Hash = std::hash<Key>,
  class KeyEqual = std::equal_to<Key>,
  class Allocator = std::allocator<Key>
> using UnorderedSet = std::unordered_set<Key, Hash, KeyEqual, Allocator>;

#endif

#endif  // FUSE_CORE__TYPES_HPP_