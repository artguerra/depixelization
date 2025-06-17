#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include "Application.h"

constexpr float ASPECT_RATIO = 16.f / 9;
constexpr int WINDOW_WIDTH = 1920;
constexpr int WINDOW_HEIGHT = WINDOW_WIDTH / ASPECT_RATIO;

// global contexts
GLFWwindow* g_window;
Application* g_app = nullptr;

// global state variables
bool g_wireframeActive = false;
bool g_mouseRightPressed = false, g_mouseLeftPressed = false;
double g_lastMouseX = 0.0f, g_lastMouseY = 0.0f;

void windowSizeCallback(GLFWwindow* window, int width, int height) {
  g_app->setWindowSize(width, height);
  glViewport(0, 0, width, height);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS && key == GLFW_KEY_X) {
    if (g_wireframeActive)
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    g_wireframeActive = !g_wireframeActive;
  } else if (action == GLFW_PRESS && (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)) {
    glfwSetWindowShouldClose(window, true);
  } else if (action == GLFW_PRESS && key == GLFW_KEY_R) {
    g_app->resetCamera();
  }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    if (action == GLFW_PRESS) {
      g_mouseRightPressed = true;
      glfwGetCursorPos(window, &g_lastMouseX, &g_lastMouseY);
    } else if (action == GLFW_RELEASE) {
      g_mouseRightPressed = false;
    }
  } else if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      double xpos, ypos;
      glfwGetCursorPos(window, &xpos, &ypos);

      g_app->handleMouseClick(xpos, ypos);

      g_mouseLeftPressed = true;
    } else if (action == GLFW_RELEASE) {
      g_mouseLeftPressed = false;
    }
  }
}

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  // prevent panning when imgui is using the mouse
  if (ImGui::GetIO().WantCaptureMouse) return;

  g_app->setMousePos(xpos, ypos);

  if (g_mouseRightPressed) {
    double dx = xpos - g_lastMouseX;
    double dy = ypos - g_lastMouseY;
    g_lastMouseX = xpos;
    g_lastMouseY = ypos;

    g_app->pan(-dx * 0.001f, dy * 0.001f);
  }

  if (g_mouseLeftPressed) {
    // handle left mouse button drag
  }
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  // prevent zooming when imgui is using the mouse
  if (ImGui::GetIO().WantCaptureMouse) return;

  if (yoffset > 0) {
    g_app->zoom(1.1f);
  } else {
    g_app->zoom(0.9f);
  }
}

void initGLFW() {
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  g_window =
      glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Interactive Depixelization", nullptr, nullptr);
  if (!g_window) {
    std::cerr << "Failed to create GLFW window" << '\n';

    glfwTerminate();
    std::exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(g_window);
  glfwSetFramebufferSizeCallback(g_window, windowSizeCallback);
  glfwSetScrollCallback(g_window, scrollCallback);
  glfwSetMouseButtonCallback(g_window, mouseButtonCallback);
  glfwSetCursorPosCallback(g_window, cursorPosCallback);
  glfwSetKeyCallback(g_window, keyCallback);
}

void initOpenGL() {
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << '\n';
    std::exit(EXIT_FAILURE);
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glPointSize(7.0f);
  glLineWidth(3.0f);
}

void initImgui() {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  // default font size is 18
  io.Fonts->AddFontFromFileTTF("external/imgui/misc/fonts/Karla-Regular.ttf", 36.0f);
  ImGui::GetStyle().ScaleAllSizes(2.0f);

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(g_window, true);
  ImGui_ImplOpenGL3_Init("#version 460");
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <image_path>" << '\n';
    return EXIT_FAILURE;
  }

  // initializations
  initGLFW();
  initOpenGL();
  initImgui();

  g_app = new Application();
  g_app->setWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

  // load image
  if (!g_app->loadImage(argv[1])) {
    std::cerr << "Failed to load image: " << argv[1] << '\n';
    return EXIT_FAILURE;
  }

  while (!glfwWindowShouldClose(g_window)) {
    glfwSwapBuffers(g_window);

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // imgui rendering
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // render
    g_app->render();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwPollEvents();
  }

  // cleanup
  delete g_app;
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwTerminate();

  return 0;
}
