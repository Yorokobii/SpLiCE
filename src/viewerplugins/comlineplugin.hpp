#ifndef COMLINEPLUGIN_HPP
#define COMLINEPLUGIN_HPP
#include <mecacellviewer/viewer.h>
#include <mecacell/utilities/grid.hpp>

struct CenterOfMassLineDrawer {
	QOpenGLShaderProgram shader;
	Cube cube;

	std::vector<std::pair<QVector3D, QVector3D>> positions;
	QVector3D center;

 public:
	template <typename R> void load(R *r) {
		shader.addShaderFromSourceCode(QOpenGLShader::Vertex,
		                               shaderWithHeader(":/shaders/mvp.vert"));
		shader.addShaderFromSourceCode(QOpenGLShader::Fragment,
		                               shaderWithHeader(":/shaders/flat.frag"));
		shader.link();
		cube.load(shader);
		center = QVector3D(0, 0, 0);
	}

	template <typename R> void call(R *r) {
		QVector3D com;
		com = toQV3D(r->getScenario().com);

		const QMatrix4x4 &view = r->getViewMatrix();
		const QMatrix4x4 &projection = r->getProjectionMatrix();

		shader.bind();
		cube.vao.bind();
		shader.setUniformValue(shader.uniformLocation("projection"), projection);
		shader.setUniformValue(shader.uniformLocation("view"), view);

		QMatrix4x4 model;
		auto AB = (com - center);
		model.translate(center + (AB)*0.5);
		auto dp = AB.normalized().x();
		if (dp != 1 && dp != -1) {
			model.rotate(acos(dp) * 180.0 / M_PI,
							QVector3D::crossProduct(QVector3D(1, 0, 0), AB));
		}
		model.scale(AB.length() * 0.5, 6.0, 6.0);
		QMatrix4x4 nmatrix = (model).inverted().transposed();
		QVector4D color(.1, 0.6, 1.0, 1.0);
		shader.setUniformValue(shader.uniformLocation("model"), model);

		shader.setUniformValue(shader.uniformLocation("color"), color);
		GL()->glDrawElements(GL_TRIANGLES, cube.indices.size(), GL_UNSIGNED_INT, 0);

		cube.vao.release();
		shader.release();
	}
};

class CenterOfMassLinePlugin {
	CenterOfMassLineDrawer cmd;

 public:
	template <typename R> void onLoad(R *renderer) {
		MenuElement<R> *nativeDisplayMenu = renderer->getDisplayMenu();
		MenuElement<R> comtrace = {"COM trace LINE", true};
		cmd.load(renderer);
		comtrace.onToggled = [&](R *r, MenuElement<R> *me) {
			if (me->isChecked())
				r->addPaintStepsMethods(156, [&](R *r2) { cmd.call(r2); });
			else
				r->erasePaintStepsMethods(156);
		};
		nativeDisplayMenu->at("Cells").add(comtrace);
	}
};

#endif
