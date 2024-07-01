#version 330 core
 
in vec4 f_Color;
out vec4 o_Color;

void main()
{
    o_Color = vec4(f_Color);
}