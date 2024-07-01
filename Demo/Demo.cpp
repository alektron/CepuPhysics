#include "pch.h"

#include "Memory/BufferPool.h"
#include "BoundingBox.h"
#include "Trees/Tree.h"
#include "Trees/Tree_SelfQueries.h"
#include "CollisionDetection/BroadPhase.h"

#include "Collidables/TypedIndex.h"
#include "Collidables/Box.h"
#include "Collidables/Shapes.h"
#include "BodySet.h"
#include "BodyDescription.h"
#include "Bodies.h"

#include <Windows.h>
#include "glew.h"
#include "gl/GL.h"

typedef BOOL (WINAPI* WGLCHOOSEPIXELFORMAT      )(HDC hdc, const int* piAttribIList, const FLOAT* pfAttribFList, UINT nMaxFormats, int* piFormats, UINT* nNumFormats);
typedef HGLRC(WINAPI* WGLCREATECONTEXTATTRIBSARB)(HDC hDC, HGLRC hShareContext, const int* attribList);
typedef BOOL (WINAPI* WGLSWAPINTERVALEXT        )(int interval);

static WGLCHOOSEPIXELFORMAT       wglChoosePixelFormatARB    = NULL;
static WGLCREATECONTEXTATTRIBSARB wglCreateContextAttribsARB = NULL;

LRESULT CALLBACK WndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

void DebugCallback(GLenum source​, GLenum type​, GLuint id​, GLenum severity​, GLsizei length​, const GLchar* message​, const void* userParam​)
{
  printf(message​);
  printf("\n");
}

bool RegisterWglFunctions()
{
  auto hInstance = GetModuleHandle(NULL);

  HWND window = CreateWindow(
    L"MyWindowClass",
    L"Temp",
    NULL,
    0, 0,
    0,
    0,
    NULL,
    NULL,
    GetModuleHandle(NULL),
    NULL
  );
  if (window == NULL) {
    printf("CreateWindow for temp window failed: %d", GetLastError());
    exit(-1);
  }

  HDC dc = GetDC(window);

  PIXELFORMATDESCRIPTOR pfd{};
  pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
  pfd.nVersion = 1;
  pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;

  int pixelformat = ChoosePixelFormat(dc, &pfd);
  if (SetPixelFormat(dc, pixelformat, nullptr) == FALSE) {
    printf("SetPixelFormat for temp device context failed");
    exit(-1);
  }

  HGLRC rc = wglCreateContext(dc);
  if (rc == NULL) {
    printf("CreateContext for temp device context failed");
    exit(-1);
  }

  if (!wglMakeCurrent(dc, rc)) {
    printf("Could not make temp context current");
    exit(-1);
  }

  wglChoosePixelFormatARB    = (WGLCHOOSEPIXELFORMAT      )wglGetProcAddress("wglChoosePixelFormatARB");
  wglCreateContextAttribsARB = (WGLCREATECONTEXTATTRIBSARB)wglGetProcAddress("wglCreateContextAttribsARB");

  if (!wglChoosePixelFormatARB   ) { printf("Could not get function ptr for wglChoosePixelFormatARB"   ); exit(-1); }
  if (!wglCreateContextAttribsARB) { printf("Could not get function ptr for wglCreateContextAttribsARB"); exit(-1); }

  DestroyWindow(window);

  return true;
}

bool ReadWholeFile(std::wstring path, std::string& o_str)
{
  HANDLE file = CreateFile(path.c_str(), GENERIC_READ, FILE_SHARE_WRITE | FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
  if (file == INVALID_HANDLE_VALUE)
    return false;

  LARGE_INTEGER size;
  GetFileSizeEx(file, &size);
  o_str.resize(size.QuadPart);
  DWORD readCnt;
  if (!ReadFile(file, o_str.data(), (DWORD)o_str.size(), &readCnt, NULL)) {
    CloseHandle(file);
    return false;
  }

  assert(readCnt == size.QuadPart);
  CloseHandle(file);
  return true;
}

using namespace CepuPhysics;

int begin = 0, end = 0;

struct Vertex
{
  glm::vec3 Pos;
  glm::vec4 Col;
};

void DrawAABB(std::vector<Vertex>* io_result, glm::vec3 min, glm::vec3 max, glm::vec4 color)
{
  Vertex a = { { min.x, min.y, min.z  }, color };
  Vertex b = { { min.x, min.y, max.z  }, color };
  Vertex c = { { max.x, min.y, max.z  }, color };
  Vertex d = { { max.x, min.y, min.z  }, color };

  Vertex e = { { min.x, max.y, min.z  }, color };
  Vertex f = { { min.x, max.y, max.z  }, color };
  Vertex g = { { max.x, max.y, max.z  }, color };
  Vertex h = { { max.x, max.y, min.z  }, color };

  const size_t NUM = 24;
  Vertex v[NUM];
  v[0] = a; v[1] = b;
  v[2] = b; v[3] = c;
  v[4] = c; v[5] = d;
  v[6] = d; v[7] = a;

  v[8] = e; v[9] = f;
  v[10] = f; v[11] = g;
  v[12] = g; v[13] = h;
  v[14] = h; v[15] = e;

  v[16] = a; v[17] = e;
  v[18] = b; v[19] = f;
  v[20] = c; v[21] = g;
  v[22] = d; v[23] = h;

  for (size_t i = 0; i < NUM; i++) {
    io_result->emplace_back(v[i]);
  }
}

void DrawCube(std::vector<Vertex>* io_result, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 e, glm::vec3 f, glm::vec3 g, glm::vec3 h, glm::vec4 color)
{
  Vertex av = { a, color };
  Vertex bv = { b, color };
  Vertex cv = { c, color };
  Vertex dv = { d, color };

  Vertex ev = { e, color };
  Vertex fv = { f, color };
  Vertex gv = { g, color };
  Vertex hv = { h, color };

  const size_t NUM = 24;
  Vertex v[NUM];
  v[0] = av; v[1] = bv;
  v[2] = bv; v[3] = cv;
  v[4] = cv; v[5] = dv;
  v[6] = dv; v[7] = av;

  v[8] = ev; v[9] = fv;
  v[10] = fv; v[11] = gv;
  v[12] = gv; v[13] = hv;
  v[14] = hv; v[15] = ev;

  v[16] = av; v[17] = ev;
  v[18] = bv; v[19] = fv;
  v[20] = cv; v[21] = gv;
  v[22] = dv; v[23] = hv;

  for (size_t i = 0; i < NUM; i++) {
    io_result->emplace_back(v[i]);
  }
}


class OverlapHandler : public IOverlapHandler
{
public:
  virtual void Handle(int32_t indexA, int32_t indexB) override
  {
    auto bodyA = m_BroadPhase->m_ActiveLeaves[indexA].GetBodyHandle();
    auto bodyB = m_BroadPhase->m_ActiveLeaves[indexB].GetBodyHandle();

    m_DebugDraw->emplace_back(m_Bodies->GetBodyRef(bodyA).GetPose().m_Position, glm::vec4(1, 0, 0, 1));
    m_DebugDraw->emplace_back(m_Bodies->GetBodyRef(bodyB).GetPose().m_Position, glm::vec4(1, 0, 0, 1));
  }

  CepuPhysics::Bodies* m_Bodies = nullptr;
  CepuPhysics::BroadPhase* m_BroadPhase = nullptr;
  std::vector<Vertex>* m_DebugDraw = nullptr;
};

int main()
{
  HWND window;

  //Create window and initialize OpenGL.
  //Not relevant for the actual text rendering
  HDC dc;
  {
    auto hInstance = GetModuleHandle(NULL);

    WNDCLASSEX wndClass;
    memset(&wndClass, 0, sizeof(wndClass));
    wndClass.cbSize        = sizeof(WNDCLASSEX);
    wndClass.style         = CS_OWNDC | CS_BYTEALIGNCLIENT | CS_DBLCLKS;
    wndClass.lpfnWndProc   = WndProc;
    wndClass.hInstance     = hInstance;
    wndClass.hIcon         = NULL;
    wndClass.hbrBackground = NULL;
    wndClass.lpszClassName = L"MyWindowClass";

    ATOM reg = RegisterClassEx(&wndClass);
    if (reg == 0) {
      printf("RegisterClass failed: %d", GetLastError());
      exit(-1);
    }

    window = CreateWindow(
      L"MyWindowClass",
      L"CepuPhysics Demo",
      WS_CAPTION| WS_SYSMENU | WS_MAXIMIZEBOX | WS_MINIMIZEBOX | WS_SIZEBOX,
      CW_USEDEFAULT,
      CW_USEDEFAULT,
      CW_USEDEFAULT,
      CW_USEDEFAULT,
      NULL,
      NULL,
      hInstance,
      NULL
    );
    if (window == NULL) {
      printf("CreateWindow failed: %d", GetLastError());
      exit(-1);
    }

    dc = GetDC(window);
    {
      RegisterWglFunctions();

#define WGL_DRAW_TO_WINDOW_ARB 0x2001
#define WGL_SUPPORT_OPENGL_ARB 0x2010
#define WGL_DOUBLE_BUFFER_ARB 0x2011
#define WGL_PIXEL_TYPE_ARB 0x2013
#define WGL_COLOR_BITS_ARB 0x2014
#define WGL_ALPHA_BITS_ARB 0x201B
#define WGL_DEPTH_BITS_ARB 0x2022
#define WGL_STENCIL_BITS_ARB 0x2023
#define WGL_TYPE_RGBA_ARB 0x202B
#define WGL_SAMPLE_BUFFERS_ARB 0x2041
#define WGL_SAMPLES_ARB 0x2042

      int  pixelFormats[1];
      UINT numFormats[1];
      int attribList[] =
      {
        WGL_DRAW_TO_WINDOW_ARB, (int)GL_TRUE,
        WGL_SUPPORT_OPENGL_ARB, (int)GL_TRUE,
        WGL_DOUBLE_BUFFER_ARB,  (int)GL_TRUE,
        WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB,
        WGL_COLOR_BITS_ARB, 8,
        WGL_ALPHA_BITS_ARB, 8,
        WGL_DEPTH_BITS_ARB, 0,
        WGL_STENCIL_BITS_ARB, 0,
        WGL_SAMPLE_BUFFERS_ARB, (int)GL_FALSE,
        WGL_SAMPLES_ARB, 0,
        0, // End
      };


      wglChoosePixelFormatARB(dc, attribList, NULL, 1, pixelFormats, numFormats);
      if (SetPixelFormat(dc, pixelFormats[0], nullptr) == FALSE) {
        printf("SetPixelFormat failed");
        exit(-1);
      }

#define WGL_CONTEXT_MAJOR_VERSION_ARB 0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB 0x2092
#define WGL_CONTEXT_LAYER_PLANE_ARB 0x2093
#define WGL_CONTEXT_FLAGS_ARB 0x2094
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001

      int attributes[] =
      {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
        WGL_CONTEXT_MINOR_VERSION_ARB, 3,
        WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
        0
      };

      HGLRC ctx = wglCreateContextAttribsARB(dc, NULL, attributes);

      if (wglMakeCurrent(dc, ctx) == FALSE)
        printf("MakeCurrent failed: %d", GetLastError());

      //VSync
      auto swapInterval = (WGLSWAPINTERVALEXT)wglGetProcAddress("wglSwapIntervalEXT");
      if (swapInterval){
        swapInterval(1);
      }

      glewExperimental = GL_TRUE;
      GLenum init = glewInit();
      GLenum err = glGetError();
      if (err != GL_NO_ERROR) {
        printf("glewInit failed: %d", err);
        exit(-1);
      }
      glEnable(GL_DEBUG_OUTPUT);
      glDebugMessageCallback(DebugCallback, nullptr);

    }

    ShowWindow(window, SW_MAXIMIZE);
  }

  GLuint vertexBuf = 0;
  glGenBuffers(1, &vertexBuf);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  glBufferData(GL_ARRAY_BUFFER, 0, 0, GL_STREAM_DRAW);

  GLuint vertexArr = 0;
  glGenVertexArrays(1, &vertexArr);
  glBindVertexArray(vertexArr);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const void*)sizeof(Vertex::Pos));
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  size_t vertexBufCapacity = 0;

  GLuint debugShader = 0;
  {
    std::string vShaderContent;
    ReadWholeFile(L"../Resources/Shaders/Debug.vs", vShaderContent);
    GLuint vShader = glCreateShader(GL_VERTEX_SHADER);

    const char* vSource = vShaderContent.c_str();
    glShaderSource(vShader, 1, &vSource, NULL);
    glCompileShader(vShader);

    std::string fShaderContent;
    ReadWholeFile(L"../Resources/Shaders/Debug.fs", fShaderContent);
    GLuint fShader = glCreateShader(GL_FRAGMENT_SHADER);

    const char* fSource = fShaderContent.c_str();
    glShaderSource(fShader, 1, &fSource, NULL);
    glCompileShader(fShader);

    debugShader = glCreateProgram();
    glAttachShader(debugShader, vShader);
    glAttachShader(debugShader, fShader);
    glLinkProgram(debugShader);

    glDeleteShader(fShader);
    glDeleteShader(vShader);
  }
  GLuint debugShader_ViewMatrix = glGetUniformLocation(debugShader, "u_ViewProj");

  std::vector<Vertex> debugDraw;
  

  CepuUtil::BufferPool bufferPool;
  CepuPhysics::BroadPhase broadPhase(bufferPool);
  CepuPhysics::Shapes shapes(&bufferPool, 128);
  CepuPhysics::Bodies bodies(&bufferPool, &shapes, &broadPhase, 128, 0);

  OverlapHandler overlapHandler;
  overlapHandler.m_Bodies = &bodies;
  overlapHandler.m_BroadPhase = &broadPhase;
  overlapHandler.m_DebugDraw = &debugDraw;

  auto boxShape = shapes.Add(CepuPhysics::Box(1, 1, 1));
  auto bigBoxShape = shapes.Add(CepuPhysics::Box(50, 50, 50));

  std::vector<BodyHandle> bodyHandles;

  const int NUM_CUBES = 50;
  int* arr = new int[7 * NUM_CUBES];
  for (size_t i = 0; i < 7 * NUM_CUBES; i++)
  {
    arr[i] = rand();
  }

  BodyDescription bodyDesc;
  for (size_t i = 0; i < NUM_CUBES; i++) {
    float rx = (float)(arr[i * 7 + 0] % 100 - 50);
    float ry = (float)(arr[i * 7 + 1] % 100 - 50);
    float rz = (float)(arr[i * 7 + 2] % 100 - 50);
    float rr = (float)(arr[i * 7 + 3] % 360);
    auto pos = glm::vec3(rx, ry, rz);
    bodyDesc.m_Pose.m_Position = pos;
    bodyDesc.m_Pose.m_Orientation = glm::angleAxis(glm::radians(rr), glm::vec3(0, 1, 0));

    float vx = (arr[i * 7 + 4] % 100 - 50) / 1000.0f;
    float vy = (arr[i * 7 + 5] % 100 - 50) / 1000.0f;
    float vz = (arr[i * 7 + 6] % 100 - 50) / 1000.0f;
    bodyDesc.m_Velocity.m_Linear = glm::vec3(vx, vy, vz);

    bodyDesc.m_Collidable.m_Shape = boxShape;
    if (i == 2)
      bodyDesc.m_Collidable.m_Shape = bigBoxShape;

    auto handle = bodies.Add(bodyDesc);
    bodyHandles.push_back(handle);
  }

  glClearColor(0.5f, 0.5f, 0.5f, 1);
  while (true)
  {
    MSG msg;
    while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) != 0) {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }

    RECT rect;
    GetClientRect(window, &rect);
    int windowWidth   = rect.right  - rect.left;
    int windowHeight =  rect.bottom - rect.top ;
    glViewport(0, 0, windowWidth, windowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::vec4 color = glm::vec4(0, 0, 1, 1);
    for (int32_t i = 0; i < broadPhase.m_ActiveTree.m_NodeCount; ++i) {
      auto& node = broadPhase.m_ActiveTree.m_Nodes[i];
      DrawAABB(&debugDraw, node.A.Min, node.A.Max, color);
      DrawAABB(&debugDraw, node.B.Min, node.B.Max, color);
      color *= 0.95f;
    }

    for (auto bodyHandle : bodyHandles) {
      auto body = bodies.GetBodyRef(bodyHandle);

      //Fake integration
      body.GetPose().m_Position += body.GetVelocity().m_Linear;
    }

    for (auto bodyHandle : bodyHandles) {
      bodies.UpdateBounds(bodyHandle);
    }

    broadPhase.Update();

    for (auto bodyHandle : bodyHandles) {
      auto body = bodies.GetBodyRef(bodyHandle);
      auto& collidable = body.GetCollidable();
      auto& state      = body.GetSolverState();
      auto& shape      = shapes.GetShape<Box>(collidable.m_Shape.GetIndex());

      glm::vec3 points[8];
      points[0] = {  shape.m_HalfHeight,  shape.m_HalfLength,  shape.m_HalfWidth };
      points[1] = {  shape.m_HalfHeight,  shape.m_HalfLength, -shape.m_HalfWidth };
      points[2] = { -shape.m_HalfHeight,  shape.m_HalfLength, -shape.m_HalfWidth };
      points[3] = { -shape.m_HalfHeight,  shape.m_HalfLength,  shape.m_HalfWidth };
      points[4] = {  shape.m_HalfHeight, -shape.m_HalfLength,  shape.m_HalfWidth };
      points[5] = {  shape.m_HalfHeight, -shape.m_HalfLength, -shape.m_HalfWidth };
      points[6] = { -shape.m_HalfHeight, -shape.m_HalfLength, -shape.m_HalfWidth };
      points[7] = { -shape.m_HalfHeight, -shape.m_HalfLength,  shape.m_HalfWidth };

      for (size_t p = 0; p < 8; p++)
        RigidPose::Transform(points[p], state.m_Motion.m_Pose, points[p]);

      DrawCube(&debugDraw, points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7], glm::vec4(0, 1, 0, 1));
    }

    CepuPhysics::GetSelfOverlaps(broadPhase.m_ActiveTree, overlapHandler);

    glBindBuffer(GL_ARRAY_BUFFER, vertexBuf);
    const size_t verticesSizeByte = sizeof(Vertex) * debugDraw.size();
    if (vertexBufCapacity < verticesSizeByte) {
      glBufferData(GL_ARRAY_BUFFER, verticesSizeByte, 0, GL_STREAM_DRAW);
      vertexBufCapacity = verticesSizeByte;
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, verticesSizeByte, debugDraw.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(debugShader);
    glm::mat4 viewProj = glm::perspective(90.f, windowWidth / (float)windowHeight, 0.01f, 10000.f) * glm::lookAt(glm::vec3(100), glm::vec3(0), glm::vec3(0, 1, 0));
    glUniformMatrix4fv(debugShader_ViewMatrix, 1, GL_FALSE, &viewProj[0][0]);
    glBindVertexArray(vertexArr);
    glDrawArrays(GL_LINES, 0, (GLsizei)debugDraw.size());
    glBindVertexArray(0);
    debugDraw.clear();

    SwapBuffers(dc);
  }
}
