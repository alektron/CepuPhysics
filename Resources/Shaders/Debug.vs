#version 330 core

layout(location = 0) in vec3 v_Position;
layout(location = 1) in vec4 v_Color;

out vec4 f_Color;

uniform mat4 u_ViewProj;

void main()
{
    gl_Position = u_ViewProj * vec4(v_Position, 1.0);
	f_Color = v_Color;
}