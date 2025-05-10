#ifndef __SHADER_H__
#define __SHADER_H__

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#include <glad/glad.h>
#include <glm/glm.hpp>

class Shader {
 private:
  GLuint m_id;

 public:
  Shader(const char *vertexPath, const char *fragmentPath) {
    std::ifstream vertexFile{vertexPath};
    std::ifstream fragmentFile{fragmentPath};

    if (!vertexFile || !fragmentFile) {
      auto &msg{std::cerr << "ERROR::SHADER::"};

      if (!vertexFile) msg << "VERTEX::";
      if (!fragmentFile) msg << "FRAGMENT::";

      msg << "FILE_NOT_READ" << std::endl;
    }

    std::stringstream vertexStream, fragmentStream;

    vertexStream << vertexFile.rdbuf();
    fragmentStream << fragmentFile.rdbuf();

    std::string vertexCode{vertexStream.str()};
    std::string fragmentCode{fragmentStream.str()};

    const char *vertexShader{vertexCode.c_str()};
    const char *fragmentShader{fragmentCode.c_str()};

    GLuint vertexId{glCreateShader(GL_VERTEX_SHADER)};
    glShaderSource(vertexId, 1, &vertexShader, NULL);
    glCompileShader(vertexId);
    checkShaderCompileErrors(vertexId, "VERTEX");

    GLuint fragmentId{glCreateShader(GL_FRAGMENT_SHADER)};
    glShaderSource(fragmentId, 1, &fragmentShader, NULL);
    glCompileShader(fragmentId);
    checkShaderCompileErrors(fragmentId, "FRAGMENT");

    m_id = glCreateProgram();
    glAttachShader(m_id, vertexId);
    glAttachShader(m_id, fragmentId);
    glLinkProgram(m_id);
    checkShaderCompileErrors(m_id, "PROGRAM");

    glDeleteShader(vertexId);
    glDeleteShader(fragmentId);
  }

  Shader(const Shader &s) : m_id{s.m_id} {}

  Shader(const Shader &&s) : m_id{std::move(s.m_id)} {}

  void use() const { glUseProgram(m_id); }

  void setBool(const std::string &name, bool value) const {
    glUniform1i(glGetUniformLocation(m_id, name.c_str()), (int)value);
  }

  void setInt(const std::string &name, int value) const {
    glUniform1i(glGetUniformLocation(m_id, name.c_str()), value);
  }

  void setFloat(const std::string &name, float value) const {
    glUniform1f(glGetUniformLocation(m_id, name.c_str()), value);
  }

  void setVec2(const std::string &name, const glm::vec2 &value) const {
    glUniform2fv(glGetUniformLocation(m_id, name.c_str()), 1, &value[0]);
  }
  void setVec2(const std::string &name, float x, float y) const {
    glUniform2f(glGetUniformLocation(m_id, name.c_str()), x, y);
  }

  void setVec3(const std::string &name, const glm::vec3 &value) const {
    glUniform3fv(glGetUniformLocation(m_id, name.c_str()), 1, &value[0]);
  }
  void setVec3(const std::string &name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(m_id, name.c_str()), x, y, z);
  }

  void setVec4(const std::string &name, const glm::vec4 &value) const {
    glUniform4fv(glGetUniformLocation(m_id, name.c_str()), 1, &value[0]);
  }
  void setVec4(const std::string &name, float x, float y, float z, float w) const {
    glUniform4f(glGetUniformLocation(m_id, name.c_str()), x, y, z, w);
  }

  void setMat2(const std::string &name, const glm::mat2 &mat) const {
    glUniformMatrix2fv(glGetUniformLocation(m_id, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  }

  void setMat3(const std::string &name, const glm::mat3 &mat) const {
    glUniformMatrix3fv(glGetUniformLocation(m_id, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  }

  void setMat4(const std::string &name, const glm::mat4 &mat) const {
    glUniformMatrix4fv(glGetUniformLocation(m_id, name.c_str()), 1, GL_FALSE, &mat[0][0]);
  }

 private:
  void checkShaderCompileErrors(GLuint shader, std::string_view type) {
    GLint success;
    GLchar infoLog[1024];
    if (type != "PROGRAM") {
      glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
      if (!success) {
        glGetShaderInfoLog(shader, 1024, NULL, infoLog);
        std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                  << infoLog << "\n -- --------------------------------------------------- -- "
                  << std::endl;
      }
    } else {
      glGetProgramiv(shader, GL_LINK_STATUS, &success);
      if (!success) {
        glGetProgramInfoLog(shader, 1024, NULL, infoLog);
        std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                  << infoLog << "\n -- --------------------------------------------------- -- "
                  << std::endl;
      }
    }
  }
};

#endif
