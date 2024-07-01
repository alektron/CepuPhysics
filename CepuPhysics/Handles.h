#pragma once

struct BodyHandle
{
  BodyHandle() = default;
  BodyHandle(int32_t value) : m_Value(value) {}

  bool operator==(BodyHandle other) const { return m_Value == other.m_Value; }

  int32_t m_Value = 0;
};

struct StaticHandle
{
  StaticHandle() = default;
  StaticHandle(int32_t value) : m_Value(value) {}

  bool operator==(StaticHandle other) const { return m_Value == other.m_Value; }

  int32_t m_Value = 0;
};
