// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "draw.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>




//#include "imgui/imgui.h"

#define BUFFER_OFFSET(x)  ((const void*) (x))

//DebugDraw g_debugDraw;
//Camera g_camera;

//
Camera::Camera()
{
	m_width = 1280;
	m_height = 800;
	ResetView();
}

//
void Camera::ResetView()
{
	m_center.Set(0.0f, 0.0f);
	m_zoom = 0.2f;
}

//
b2Vec2 Camera::ConvertScreenToWorld(const b2Vec2& ps)
{
	float w = float(m_width);
	float h = float(m_height);
	float u = ps.x / w;
	float v = (h - ps.y) / h;

	float ratio = w / h;
	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= m_zoom;

	b2Vec2 lower = m_center - extents;
	b2Vec2 upper = m_center + extents;

	b2Vec2 pw;
	pw.x = (1.0f - u) * lower.x + u * upper.x;
	pw.y = (1.0f - v) * lower.y + v * upper.y;
	return pw;
}

//
b2Vec2 Camera::ConvertWorldToScreen(const b2Vec2& pw)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= m_zoom;

	b2Vec2 lower = m_center - extents;
	b2Vec2 upper = m_center + extents;

	float u = (pw.x - lower.x) / (upper.x - lower.x);
	float v = (pw.y - lower.y) / (upper.y - lower.y);

	b2Vec2 ps;
	ps.x = u * w;
	ps.y = (1.0f - v) * h;
	return ps;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
void Camera::BuildProjectionMatrix(float* m, float zBias)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= m_zoom;

	b2Vec2 lower = m_center - extents;
	b2Vec2 upper = m_center + extents;

	m[0] = 2.0f / (upper.x - lower.x);
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 2.0f / (upper.y - lower.y);
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = 1.0f;
	m[11] = 0.0f;

	m[12] = -(upper.x + lower.x) / (upper.x - lower.x);
	m[13] = -(upper.y + lower.y) / (upper.y - lower.y);
	m[14] = zBias;
	m[15] = 1.0f;
}

//
static void sCheckGLError()
{
	GLenum errCode = glGetError();
	if (errCode != GL_NO_ERROR)
	{
		fprintf(stderr, "OpenGL error = %d\n", errCode);
		assert(false);
	}
}

// Prints shader compilation errors
static void sPrintLog(GLuint object)
{
	GLint log_length = 0;
	if (glIsShader(object))
		glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else if (glIsProgram(object))
		glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else
	{
		fprintf(stderr, "printlog: Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(object))
		glGetShaderInfoLog(object, log_length, NULL, log);
	else if (glIsProgram(object))
		glGetProgramInfoLog(object, log_length, NULL, log);

	fprintf(stderr, "%s", log);
	free(log);
}


//
static GLuint sCreateShaderFromString(const char* source, GLenum type)
{
	GLuint res = glCreateShader(type);
	const char* sources[] = { source };
	glShaderSource(res, 1, sources, NULL);
	glCompileShader(res);
	GLint compile_ok = GL_FALSE;
	glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
	if (compile_ok == GL_FALSE)
	{
		fprintf(stderr, "Error compiling shader of type %d!\n", type);
		sPrintLog(res);
		glDeleteShader(res);
		return 0;
	}

	return res;
}

// 
static GLuint sCreateShaderProgram(const char* vs, const char* fs)
{
	GLuint vsId = sCreateShaderFromString(vs, GL_VERTEX_SHADER);
	GLuint fsId = sCreateShaderFromString(fs, GL_FRAGMENT_SHADER);
	assert(vsId != 0 && fsId != 0);

	GLuint programId = glCreateProgram();
	glAttachShader(programId, vsId);
	glAttachShader(programId, fsId);
	glBindFragDataLocation(programId, 0, "color");
	glLinkProgram(programId);

	glDeleteShader(vsId);
	glDeleteShader(fsId);

	GLint status = GL_FALSE;
	glGetProgramiv(programId, GL_LINK_STATUS, &status);
	assert(status != GL_FALSE);

	return programId;
}

//
struct GLRenderPoints
{
	void Create()
	{
		const char* vs = \
			"#version 330\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"layout(location = 2) in float v_size;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"   gl_PointSize = v_size;\n"
			"}\n";

		const char* fs = \
			"#version 330\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;
		m_sizeAttribute = 2;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(3, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);
		glEnableVertexAttribArray(m_sizeAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_sizes), m_sizes, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(3, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2& v, const b2Color& c, float size)
	{
		if (m_count == e_maxVertices)
			Flush();

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		m_sizes[m_count] = size;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.0f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(float), m_sizes);

		glEnable(GL_PROGRAM_POINT_SIZE);
		glDrawArrays(GL_POINTS, 0, m_count);
		glDisable(GL_PROGRAM_POINT_SIZE);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum { e_maxVertices = 512 };
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];
	float m_sizes[e_maxVertices];

	int32 m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[3];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
	GLint m_sizeAttribute;
};

//
struct GLRenderLines
{
	void Create()
	{
		const char* vs = \
			"#version 330\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 330\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2& v, const b2Color& c)
	{
		if (m_count == e_maxVertices)
			Flush();

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.1f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArrays(GL_LINES, 0, m_count);
		glDisable(GL_BLEND);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum { e_maxVertices = 2 * 512 };
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];

	int32 m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};

//
struct GLRenderTriangles
{
	void Create()
	{
		const char* vs = \
			"#version 330\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 330\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2& v, const b2Color& c)
	{
		if (m_count == e_maxVertices)
			Flush();

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.2f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArrays(GL_TRIANGLES, 0, m_count);
		glDisable(GL_BLEND);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum { e_maxVertices = 3 * 512 };
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];

	int32 m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};

//
DebugDraw::DebugDraw()
{
	m_showUI = true;
	m_points = NULL;
	m_lines = NULL;
	m_triangles = NULL;
}

//
DebugDraw::~DebugDraw()
{
	b2Assert(m_points == NULL);
	b2Assert(m_lines == NULL);
	b2Assert(m_triangles == NULL);
}

//
void DebugDraw::Create()
{
	m_points = new GLRenderPoints;
	m_points->Create();
	m_lines = new GLRenderLines;
	m_lines->Create();
	m_triangles = new GLRenderTriangles;
	m_triangles->Create();
    
}

//
void DebugDraw::Destroy()
{
	m_points->Destroy();
	delete m_points;
	m_points = NULL;

    m_lines->Destroy();
    delete m_lines;
	m_lines = NULL;

	m_triangles->Destroy();
    delete m_triangles;
    m_triangles = NULL;
}

//
void DebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	b2Vec2 p1 = vertices[vertexCount - 1];
	for (int32 i = 0; i < vertexCount; ++i)
	{
		b2Vec2 p2 = vertices[i];
		m_lines->Vertex(p1, color);
		m_lines->Vertex(p2, color);
		p1 = p2;
	}
}

//
void DebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	b2Color fillColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);

	for (int32 i = 1; i < vertexCount - 1; ++i)
	{
		m_triangles->Vertex(vertices[0], fillColor);
		m_triangles->Vertex(vertices[i], fillColor);
		m_triangles->Vertex(vertices[i + 1], fillColor);
	}

	b2Vec2 p1 = vertices[vertexCount - 1];
	for (int32 i = 0; i < vertexCount; ++i)
	{
		b2Vec2 p2 = vertices[i];
		m_lines->Vertex(p1, color);
		m_lines->Vertex(p2, color);
		p1 = p2;
	}
}
void DebugDraw::DrawSquare(const b2Vec2 centre, float size, const b2Color& color)
{
	b2Color fillColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);

    float s2 = size / 2;
    m_triangles->Vertex(centre + b2Vec2(-s2, -s2), color);
    m_triangles->Vertex(centre + b2Vec2(-s2, s2), color);
    m_triangles->Vertex(centre + b2Vec2(s2, s2), color);
    m_triangles->Vertex(centre + b2Vec2(-s2, -s2), color);
    m_triangles->Vertex(centre + b2Vec2(s2, s2), color);
    m_triangles->Vertex(centre + b2Vec2(s2, -s2), color);

}


//
void DebugDraw::DrawCircle(const b2Vec2& center, float radius, const b2Color& color)
{
	const float k_segments = 32.0f;
	const float k_increment = 2.0f * b2_pi / k_segments;
	float sinInc = sinf(k_increment);
	float cosInc = cosf(k_increment);
	b2Vec2 r1(1.0f, 0.0f);
	b2Vec2 v1 = center + radius * r1;
	for (int32 i = 0; i < k_segments; ++i)
	{
		// Perform rotation to avoid additional trigonometry.
		b2Vec2 r2;
		r2.x = cosInc * r1.x - sinInc * r1.y;
		r2.y = sinInc * r1.x + cosInc * r1.y;
		b2Vec2 v2 = center + radius * r2;
		m_lines->Vertex(v1, color);
		m_lines->Vertex(v2, color);
		r1 = r2;
		v1 = v2;
	}
}

//
void DebugDraw::DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color)
{
	const float k_segments = 32.0f;
	const float k_increment = 2.0f * b2_pi / k_segments;
	float sinInc = sinf(k_increment);
	float cosInc = cosf(k_increment);
	b2Vec2 v0 = center;
	b2Vec2 r1(cosInc, sinInc);
	b2Vec2 v1 = center + radius * r1;
	b2Color fillColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	for (int32 i = 0; i < k_segments; ++i)
	{
		// Perform rotation to avoid additional trigonometry.
		b2Vec2 r2;
		r2.x = cosInc * r1.x - sinInc * r1.y;
		r2.y = sinInc * r1.x + cosInc * r1.y;
		b2Vec2 v2 = center + radius * r2;
		m_triangles->Vertex(v0, fillColor);
		m_triangles->Vertex(v1, fillColor);
		m_triangles->Vertex(v2, fillColor);
		r1 = r2;
		v1 = v2;
	}

	r1.Set(1.0f, 0.0f);
	v1 = center + radius * r1;
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 r2;
		r2.x = cosInc * r1.x - sinInc * r1.y;
		r2.y = sinInc * r1.x + cosInc * r1.y;
		b2Vec2 v2 = center + radius * r2;
		m_lines->Vertex(v1, color);
		m_lines->Vertex(v2, color);
		r1 = r2;
		v1 = v2;
	}

	// Draw a line fixed in the circle to animate rotation.
	// b2Vec2 p = center + radius * axis;
	// m_lines->Vertex(center, color);
	// m_lines->Vertex(p, color);
}


void DebugDraw::DrawFont(b2Vec2 p, float scale, const char* string, int mode, const b2Color& color)
{
    wchar_t buffer[1024];
    int len = mbstowcs(buffer, string, 1024);
    float xpos = 0;
    ttf_mesh_t *out;

    // Use 'M' character width for space width
    float space_adv = font->glyphs[ttf_find_glyph(font, L'M')].advance;
    float asc = font->os2.sTypoAscender;
    float dsc = font->os2.sTypoDescender;

    if (mode == 1)
    {
        // Centered, assume fixed width
        p.x -= len * scale * space_adv * 0.5;
        p.y -= (asc + dsc) * scale * 0.5;
    }

    for(int j = 0; j < len; j++)
    {
        float xincr;
        int index = ttf_find_glyph(font, buffer[j]);
        if (string[j] == 32)
        {
            // Assume space ch
            xincr = space_adv;
        }
        else
        {
            int ret = ttf_glyph2mesh(&font->glyphs[index], &out, TTF_QUALITY_NORMAL, TTF_FEATURES_DFLT);
            if (ret != TTF_DONE)
            {
                //printf("Bad font %d\n", ret);
                continue;
            }
            xincr = font->glyphs[index].advance;
            // b2Color c(0.5, 0.5, 0.5);
            //b2Vec2 org(x, y);
            for(int i = 0; i < out->nfaces; i++)
            {
                int i1 = out->faces[i].v1;
                int i2 = out->faces[i].v2;
                int i3 = out->faces[i].v3;
                b2Vec2 v1 = p + scale * b2Vec2(xpos + out->vert[i1].x, out->vert[i1].y);
                b2Vec2 v2 = p + scale * b2Vec2(xpos + out->vert[i2].x, out->vert[i2].y);
                b2Vec2 v3 = p + scale * b2Vec2(xpos + out->vert[i3].x, out->vert[i3].y);
                m_triangles->Vertex(v1, color);
                m_triangles->Vertex(v2, color);
                m_triangles->Vertex(v3, color);
                // printf("adv %f v1 %f %f %f %f %f %f\n", xincr, v1.x, v1.y, v2.x, v2.y, v3.x, v3.y);
            }
            ttf_free_mesh(out);

        }
        xpos += xincr;
    }
}

void DebugDraw::DrawShadedCircle(const b2Vec2& center, float radius, const b2Color& color)
{
    const float k_segments = 32.0f;
    const float k_increment = 2.0f * b2_pi / k_segments;
    float sinInc = sinf(k_increment);
    float cosInc = cosf(k_increment);
    b2Vec2 v0 = center;
    b2Vec2 r1(cosInc, sinInc);
    b2Vec2 v1 = center + radius * r1;
    b2Color transparent_color(color.r, color.g, color.b, 0.0f);
    for (int32 i = 0; i < k_segments; ++i)
    {
        // Perform rotation to avoid additional trigonometry.
        b2Vec2 r2;
        r2.x = cosInc * r1.x - sinInc * r1.y;
        r2.y = sinInc * r1.x + cosInc * r1.y;
        b2Vec2 v2 = center + radius * r2;
        m_triangles->Vertex(v0, color);
        m_triangles->Vertex(v1, transparent_color);
        m_triangles->Vertex(v2, transparent_color);
        r1 = r2;
        v1 = v2;
    }
}



//
void DebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
	m_lines->Vertex(p1, color);
	m_lines->Vertex(p2, color);
}


void DebugDraw::DrawThickSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color, const float width)
{
    b2Vec2 d = p2 - p1;
    float w2 = width * 0.5;
    float th = atan2f(d.y, d.x);
    b2Vec2 v = w2 * b2Vec2(cos(th + M_PI/2), sin(th + M_PI/2));
    b2Vec2 v0 = p1 + v;
    b2Vec2 v1 = p1 - v;
    b2Vec2 v2 = p2 + v;
    b2Vec2 v3 = p2 - v;
    m_triangles->Vertex(v0, color);
    m_triangles->Vertex(v1, color);
    m_triangles->Vertex(v2, color);
    m_triangles->Vertex(v1, color);
    m_triangles->Vertex(v2, color);
    m_triangles->Vertex(v3, color);
}





//
void DebugDraw::DrawTransform(const b2Transform& xf)
{
	const float k_axisScale = 0.4f;
	b2Color red(1.0f, 0.0f, 0.0f);
	b2Color green(0.0f, 1.0f, 0.0f);
	b2Vec2 p1 = xf.p, p2;

	m_lines->Vertex(p1, red);
	p2 = p1 + k_axisScale * xf.q.GetXAxis();
	m_lines->Vertex(p2, red);

	m_lines->Vertex(p1, green);
	p2 = p1 + k_axisScale * xf.q.GetYAxis();
	m_lines->Vertex(p2, green);
}

//
void DebugDraw::DrawPoint(const b2Vec2& p, float size, const b2Color& color)
{
	m_points->Vertex(p, color, size);
}

//
void DebugDraw::DrawString(int x, int y, const char* string, ...)
{
	if (m_showUI == false)
	{
		return;
	}

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
	ImGui::SetCursorPos(ImVec2(float(x), float(y)));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

//
void DebugDraw::DrawString(const b2Vec2& pw, const char* string, ...)
{
	b2Vec2 ps = g_camera.ConvertWorldToScreen(pw);

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps.x, ps.y));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

//
void DebugDraw::DrawAABB(b2AABB* aabb, const b2Color& c)
{
	b2Vec2 p1 = aabb->lowerBound;
	b2Vec2 p2 = b2Vec2(aabb->upperBound.x, aabb->lowerBound.y);
	b2Vec2 p3 = aabb->upperBound;
	b2Vec2 p4 = b2Vec2(aabb->lowerBound.x, aabb->upperBound.y);

	m_lines->Vertex(p1, c);
	m_lines->Vertex(p2, c);

	m_lines->Vertex(p2, c);
	m_lines->Vertex(p3, c);

	m_lines->Vertex(p3, c);
	m_lines->Vertex(p4, c);

	m_lines->Vertex(p4, c);
	m_lines->Vertex(p1, c);
}

//

void DebugDraw::DrawDigit(const b2Vec2& p1, int d, float s, const b2Color& color)
{
    //   -0-
    //  5   1
    //   -6-
    //  4   2
    //   -3-  
    //                        0     1     2     3     4     5     6     7     8     9
    const char data[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};

    int x = data[d];
    for(int i = 0; i < 7; i++, x>>=1)
        if (x & 1)
            switch(i)
            {
                case 0:
                    m_lines->Vertex(p1+b2Vec2(-s, s*2), color);
                    m_lines->Vertex(p1+b2Vec2( s, s*2), color);
                    break;
                case 1:
                    m_lines->Vertex(p1+b2Vec2( s, s*2), color);
                    m_lines->Vertex(p1+b2Vec2( s,   0), color);
                    break;
                case 2:
                    m_lines->Vertex(p1+b2Vec2( s,   0), color);
                    m_lines->Vertex(p1+b2Vec2( s,-s*2), color);
                    break;
                case 3:
                    m_lines->Vertex(p1+b2Vec2(-s,-s*2), color);
                    m_lines->Vertex(p1+b2Vec2( s,-s*2), color);
                    break;
                case 4:
                    m_lines->Vertex(p1+b2Vec2(-s,   0), color);
                    m_lines->Vertex(p1+b2Vec2(-s,-s*2), color);
                    break;
                case 5:
                    m_lines->Vertex(p1+b2Vec2(-s,   0), color);
                    m_lines->Vertex(p1+b2Vec2(-s, s*2), color);
                    break;
                case 6:
                    m_lines->Vertex(p1+b2Vec2(-s,   0), color);
                    m_lines->Vertex(p1+b2Vec2( s,   0), color);
                    break;
            }
}


void DebugDraw::Flush()
{
	m_triangles->Flush();
	m_lines->Flush();
	m_points->Flush();
}