#pragma once

#include <Eigen/Dense>
#include <vector>

#include <TColgp_HArray1OfPnt2d.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Lin2d.hxx>

#include <Geom2d_TrimmedCurve.hxx>
#include <GC_MakeSegment.hxx>

#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge2d.hxx>
#include <Geom_Curve.hxx>
#include <Geom2d_Curve.hxx>
#include <BRep_Tool.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <Geom2d_Ellipse.hxx>

#include <Geom2d_Circle.hxx>


using namespace Eigen;
using namespace std;

#define FITTINGTOLERANCE 0.2

enum CurveType {
	LineCurve,
	CircleCurve,
	EclipseCurve,
	BSplineCurve
};


enum FittingType {
	Fitting2D,
	Fitting3D
};

struct FitCurve
{
	CurveType curveType;
	TopoDS_Edge edge;
	tuple<gp_Pnt2d, gp_Pnt2d> line;
	tuple<gp_Pnt2d, double, double, double> circle;	// 圆心，半径，开始角度，终止角度
	tuple<gp_Pnt2d, double, double, double, double, double> eclipse;	//圆心，a，b，x轴偏移角, 开始角度，终止角度
};

class LineFitting
{
public:
	LineFitting() {}
	LineFitting(const vector<double>& pts, const FittingType& fittingType = Fitting2D)
	{
		Init(pts, fittingType);
	}
	LineFitting(const Handle(TColgp_HArray1OfPnt2d)& aPoints, const FittingType& fittingType = Fitting2D)
	{
		Init(aPoints, fittingType);
	}
	void Init(const vector<double>& pts, const FittingType& fittingType = Fitting2D)
	{
		this->fittingType = fittingType;

		if (fittingType == Fitting2D)
		{
			for (int i = 0; i < pts.size(); i += 2)
			{
				Vector2d pt(pts[i], pts[i + 1]);
				samplingPoints.push_back(pt);
			}
		}
	}
	void Init(const Handle(TColgp_HArray1OfPnt2d)& aPoints, const FittingType& fittingType = Fitting2D)
	{
		this->fittingType = fittingType;

		if (fittingType == Fitting2D)
		{
			for (int i = 1; i <= aPoints->Size(); i++)
			{
				Vector2d pt(aPoints->Value(i).X(), aPoints->Value(i).Y());
				samplingPoints.push_back(pt);
			}
		}
	}

	bool Perform()
	{
		if (samplingPoints.size() == 2)
		{
			x1 = samplingPoints[0].x();
			y1 = samplingPoints[0].y();
			x2 = samplingPoints[1].x();
			y2 = samplingPoints[1].y();

			return true;
		}
		else
		{
			x1 = samplingPoints[0].x();
			y1 = samplingPoints[0].y();

			double distance = (samplingPoints[0] - samplingPoints[samplingPoints.size() - 1]).norm();
			if (distance < 1e-6)
			{
				x2 = samplingPoints[samplingPoints.size() - 2].x();
				y2 = samplingPoints[samplingPoints.size() - 2].y();
			}
			else
			{
				x2 = samplingPoints[samplingPoints.size() - 1].x();
				y2 = samplingPoints[samplingPoints.size() - 1].y();
			}
		}

		if (fittingType == Fitting2D)
		{
			return CheckError();
		}

		return true;
	}

	void GetLineParameter(double p1[2], double p2[2])
	{
		p1[0] = x1;
		p1[0] = y1;
		p2[1] = x2;
		p2[1] = y2;
	}

	void GetLineParameter(gp_Pnt2d &p1, gp_Pnt2d &p2)
	{
		p1.SetCoord(x1, y1);
		p2.SetCoord(x2, y2);
	}

private:
	FittingType fittingType;
	vector<Vector2d> samplingPoints;

	double x1, x2, y1, y2;

	bool CheckError()
	{
		Vector2d p1 = samplingPoints[0];
		Vector2d p2 = samplingPoints[samplingPoints.size() - 1];

		double distance = (p1 - p2).norm();
	
		if (distance < 1e-6)
		{
			p2 = samplingPoints[samplingPoints.size() - 2];
		}

		for (int i = 1; i < samplingPoints.size() - 1; i++)
		{
			Vector2d p0 = samplingPoints[i];

			double e = (p2.y() - p1.y()) * (p2.x() - p0.x()) - (p2.y() - p0.y()) * (p2.x() - p1.x()); 
			if (fabs(e) > FITTINGTOLERANCE)
			{
				return false;
			}
		}

		return true;
	}
};


class CircleFitting {

	// 参考 https://www.cnblogs.com/xiaxuexiaoab/p/16276402.html

public:
	CircleFitting(){}
	CircleFitting(const vector<double>& pts, const FittingType& fittingType = Fitting2D)
	{
		Init(pts, fittingType);
	}
	CircleFitting(const Handle(TColgp_HArray1OfPnt2d)& aPoints, const FittingType& fittingType = Fitting2D)
	{
		Init(aPoints, fittingType);
	}
	~CircleFitting() 
	{ 
		samplingPoints.clear(); 
		samplingPoints3D.clear();
	}	

	void Init(const Handle(TColgp_HArray1OfPnt2d)& aPoints, const FittingType& fittingType = Fitting2D)
	{
		this->fittingType = fittingType;

		if (fittingType == Fitting2D)
		{
			for (int i = 1; i <= aPoints->Size(); i++)
			{
				Vector2d pt(aPoints->Value(i).X(), aPoints->Value(i).Y());
				samplingPoints.push_back(pt);
			}
		}
	}

	void Init(const vector<double>& pts, const FittingType& fittingType = Fitting2D)
	{
		this->fittingType = fittingType;

		if (fittingType == Fitting2D)
		{
			for (int i = 0; i < pts.size(); i += 2)
			{
				Vector2d pt(pts[i], pts[i + 1]);
				samplingPoints.push_back(pt);
			}
		}
		else
		{
			for (int i = 0; i < pts.size(); i += 3)
			{
				Vector3d pt(pts[i], pts[i + 1], pts[i + 2]);
				samplingPoints3D.push_back(pt);
			}
		}
	}

    bool Perform()
    {
		if (fittingType == Fitting2D)
		{
			LeastSquareCircleFitting();
			return CheckError();
		}
		else
		{
			LeastSquareCircleFitting3D();
			return CheckError3D();
		}
    }

	void GetCircleParameter(double center[2], double& R)
	{
		center[0] = this->Xc;
		center[1] = this->Yc;
		R = this->R;
	}

	void GetCircleParameter(gp_Pnt2d &center, double& R, gp_Pnt2d &p1, gp_Pnt2d& p2)
	{
		center.SetCoord(Xc, Yc);
		R = this->R;
		p1.SetCoord(samplingPoints[0].x(), samplingPoints[0].y());
		p2.SetCoord(samplingPoints[samplingPoints.size() - 1].x(), samplingPoints[samplingPoints.size() -1].y());
	}

	void GetCircleParameter3D(double center[3], double normal[3], double& R)
	{
		center[0] = this->Xc;
		center[1] = this->Yc;
		center[2] = this->Zc;

		normal[0] = nor.x();
		normal[1] = nor.y();
		normal[2] = nor.z();

		R = this->R;
	}

private:

	FittingType fittingType;
	vector<Vector2d> samplingPoints;
	vector<Vector3d> samplingPoints3D;

	double Xc, Yc, Zc, R;
	Vector3d nor;	

private:

	void LeastSquareCircleFitting()
	{
		int N = samplingPoints.size();

		double sumX = 0.0;
		double sumY = 0.0;
		double sumX2 = 0.0;
		double sumY2 = 0.0;
		double sumX3 = 0.0;
		double sumY3 = 0.0;
		double sumXY = 0.0;
		double sumXY2 = 0.0;
		double sumX2Y = 0.0;

		for (int pId = 0; pId < N; ++pId) {
			sumX += samplingPoints[pId].x();
			sumY += samplingPoints[pId].y();

			double x2 = samplingPoints[pId].x() * samplingPoints[pId].x();
			double y2 = samplingPoints[pId].y() * samplingPoints[pId].y();
			sumX2 += x2;
			sumY2 += y2;

			sumX3 += x2 * samplingPoints[pId].x();
			sumY3 += y2 * samplingPoints[pId].y();
			sumXY += samplingPoints[pId].x() * samplingPoints[pId].y();
			sumXY2 += samplingPoints[pId].x() * y2;
			sumX2Y += x2 * samplingPoints[pId].y();
		}

		double C, D, E, G, H;
		double a, b, c;

		C = N * sumX2 - sumX * sumX;
		D = N * sumXY - sumX * sumY;
		E = N * sumX3 + N * sumXY2 - (sumX2 + sumY2) * sumX;
		G = N * sumY2 - sumY * sumY;
		H = N * sumX2Y + N * sumY3 - (sumX2 + sumY2) * sumY;

		a = (H * D - E * G) / (C * G - D * D);
		b = (H * C - E * D) / (D * D - G * C);
		c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

		Xc = -a / 2.0;
		Yc = -b / 2.0;
		R = sqrt(a * a + b * b - 4 * c) / 2.0;
	}

	void LeastSquareCircleFitting3D()
    {
        vector<double> circle;

        int num = samplingPoints3D.size();
        int dim = 3;

        Eigen::MatrixXd M(num, dim);

        for (int i = 0; i < num; i++)
        {
			M(i, 0) = samplingPoints3D[i].x();
			M(i, 1) = samplingPoints3D[i].y();
			M(i, 2) = samplingPoints3D[i].z();
        }

        Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);
        Eigen::Vector3d A = (M.transpose() * M).inverse() * M.transpose() * L1;

        nor = A;		

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);

        for (int i = 0; i < num - 1; i++)
        {
            B.row(i) = M.row(i + 1) - M.row(i);
        }

        Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
        for (int i = 0; i < num - 1; i++)
        {
            L2(i) = (M(i + 1, 0) * M(i + 1, 0) + M(i + 1, 1) * M(i + 1, 1) + M(i + 1, 2) * M(i + 1, 2)
                - (M(i, 0) * M(i, 0) + M(i, 1) * M(i, 1) + M(i, 2) * M(i, 2))) / 2.0;
        }

        Eigen::Matrix4d D;
        D.setZero();
        D.block<3, 3>(0, 0) = B.transpose() * B;
        D.block<3, 1>(0, 3) = A;
        D.block<1, 3>(3, 0) = A.transpose();

        Eigen::Vector4d L3((B.transpose() * L2)(0), (B.transpose() * L2)(1), (B.transpose() * L2)(2), 1);
        Eigen::Vector4d C = D.inverse() * L3;

        double radius = 0;
        for (int i = 0; i < num; i++)
        {
            Eigen::Vector3d tmp(M.row(i)(0) - C(0), M.row(i)(1) - C(1), M.row(i)(2) - C(2));
            radius = radius + sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1) + tmp(2) * tmp(2));
        }

		nor.normalize();       
		
		Xc = C(0);
		Yc = C(1);
		Zc = C(2);

		R = radius / num;
    }

    bool CheckError3D()
    {
		for (int i = 0; i < samplingPoints3D.size(); i++)
		{
			double e = (Xc - samplingPoints3D[i].x()) * (Xc - samplingPoints3D[i].x()) +
				(Yc - samplingPoints3D[i].y()) * (Yc - samplingPoints3D[i].y()) +
				(Zc - samplingPoints3D[i].z()) * (Zc - samplingPoints3D[i].z()) - R * R;

			if (e > FITTINGTOLERANCE)
			{
				return false;
			}
		}

		return true;
    }

	bool CheckError()
	{
		for (int i = 0; i < samplingPoints.size(); i++)
		{
			double e = (Xc - samplingPoints[i].x()) * (Xc - samplingPoints[i].x()) +
				(Yc - samplingPoints[i].y()) * (Yc - samplingPoints[i].y()) - R * R;

			if (fabs(e) > FITTINGTOLERANCE)
			{
				return false;
			}
		}

		return true;
	}
};


class EclipseFitting {

	// 参考 https://blog.csdn.net/zh471021698/article/details/108812050

public:
	EclipseFitting() {}
	EclipseFitting(const vector<double>& pts)
	{
		Init(pts);
	}
	EclipseFitting(const Handle(TColgp_HArray1OfPnt2d)& aPoints)
	{
		Init(aPoints);
	}
	~EclipseFitting() { samplingPoints.clear(); }

	void Init(const vector<double>& pts)
	{
		for (int i = 0; i < pts.size(); i+=2)
		{
			Vector2d pt(pts[i], pts[i + 1]);
			samplingPoints.push_back(pt);
		}
	}
	void Init(const Handle(TColgp_HArray1OfPnt2d)& aPoints)
	{
		for (int i = 1; i <= aPoints->Size(); i++)
		{
			Vector2d pt(aPoints->Value(i).X(), aPoints->Value(i).Y());
			samplingPoints.push_back(pt);
			}
	}
	bool Perform()
	{
		LeastSquareEclipseFitting();
		return CheckError();
	}

	void GetEclipseParameter(double center[2], double& a, double& b, double& theta)
	{
		center[0] = this->Xc;
		center[1] = this->Yc;
		a = this->a;
		b = this->b;
		theta = this->theta;
	}

	void GetEclipseParameter(gp_Pnt2d &center, double& a, double& b, double& theta, gp_Pnt2d& p1, gp_Pnt2d& p2)
	{
		center.SetCoord(Xc, Yc);
		a = this->a;
		b = this->b;
		theta = this->theta;

		p1.SetCoord(samplingPoints[0].x(), samplingPoints[0].y());
		p2.SetCoord(samplingPoints[samplingPoints.size() - 1].x(), samplingPoints[samplingPoints.size() - 1].y());
	}

	void GetEclipseParameter(double coef[5], gp_Pnt2d &p1, gp_Pnt2d& p2)
	{
		coef[0] = A;
		coef[1] = B;
		coef[2] = C;
		coef[3] = D;
		coef[4] = E;

		p1.SetCoord(samplingPoints[0].x(), samplingPoints[0].y());
		p2.SetCoord(samplingPoints[samplingPoints.size() - 1].x(), samplingPoints[samplingPoints.size() - 1].y());
	}

private:
	double A, B, C, D, E;
	double a, b, Xc, Yc, theta;

    vector<Vector2d> samplingPoints;

    void LeastSquareEclipseFitting()
    {
		A = 0.00, B = 0.00, C = 0.00, D = 0.00, E = 0.00;
		double x2y2 = 0.0, x1y3 = 0.0, x2y1 = 0.0, x1y2 = 0.0, x1y1 = 0.0, yyy4 = 0.0, yyy3 = 0.0, yyy2 = 0.0, xxx2 = 0.0, xxx1 = 0.0, yyy1 = 0.0, x3y1 = 0.0, xxx3 = 0.0, N = 0.0;
		//for (int i = 0; i < 6; i++)
		for (int i = 0; i < samplingPoints.size(); i++)
		{
			double xi = samplingPoints[i].x(), yi = samplingPoints[i].y();

			x2y2 += xi * xi * yi * yi;
			x1y3 += xi * yi * yi * yi;
			x2y1 += xi * xi * yi;
			x1y2 += xi * yi * yi;
			x1y1 += xi * yi;
			yyy4 += yi * yi * yi * yi;
			yyy3 += yi * yi * yi;
			yyy2 += yi * yi;
			xxx2 += xi * xi;
			xxx1 += xi;
			yyy1 += yi;
			x3y1 += xi * xi * xi * yi;
			xxx3 += xi * xi * xi;
		}

		N = double(samplingPoints.size());

		Matrix<double, 5, 5> M1;
		M1 << x2y2, x1y3, x2y1, x1y2, x1y1, x1y3, yyy4, x1y2, yyy3, yyy2, x2y1, x1y2, xxx2, x1y1, xxx1, x1y2, yyy3, x1y1, yyy2, yyy1, x1y1, yyy2, xxx1, yyy1, N;

		VectorXd M2(5);
		M2 << x3y1, x2y2, xxx3, x2y1, xxx2;

		M2 = -M2;

		VectorXd X(5);

		// X = (M1.transpose() * M1).inverse() * M1.transpose() * M2; // 这个公式虽然与下面公式等价，但是由于计算的是病态矩阵，计算时出错

		X = M1.inverse() * M2;


		A = X(0);
		B = X(1);
		C = X(2);
		D = X(3);
		E = X(4);

		///求拟合结果重要参数

		double Xp = (A * D - 2 * B * C) / (A * A - 4 * B);
		double Yp = (A * C - 2 * D) / (A * A - 4 * B);
		double theta_r = 0.5 * atan(A / (B - 1));

		Xc = -Xp;
		Yc = -Yp;
		
		theta = -theta_r;
		a = sqrt((Xp * Xp + A * Xp * Yp + B * Yp * Yp - E) / (cos(theta_r) * cos(theta_r) - A * sin(theta_r) * cos(theta_r) + B * sin(theta_r) * sin(theta_r)));
		b = sqrt((Xp * Xp + A * Xp * Yp + B * Yp * Yp - E) / (sin(theta_r) * sin(theta_r) + A * sin(theta_r) * cos(theta_r) + B * cos(theta_r) * cos(theta_r)));
    }

	bool CheckError()
	{
		//方程为：X^2+A*XY+B*Y^2+C*X+D*Y+E=0
		for (int i = 0; i < samplingPoints.size(); i++)
		{
			double xi = samplingPoints[i].x();
			double yi = samplingPoints[i].y();

			double e = xi * xi + A * xi * yi + B * yi * yi + C * xi + D * yi + E;
			
			if (fabs(e) > FITTINGTOLERANCE)
			{
				return false;
			}
		}
		return true;
	}
};


class SheetFlattenFitting
{
public:
	SheetFlattenFitting() {}
	~SheetFlattenFitting() {}
	int Perform(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve);

private:
	CurveType curveType;

	bool FitLine(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve);
	bool FitCircle(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve);
	bool FitEclipse(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve);
	bool FitBSpline(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve);
	bool IsPointOnCurve2d(const gp_Pnt2d& pt, const Handle(Geom2d_Curve)& curve);
};



inline int SheetFlattenFitting::Perform(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve)
{
	if (FitLine(aPoints, fitCurve))
	{
		return 0;
	}
	else if (FitCircle(aPoints, fitCurve))
	{
		return 1;
	}
	else if (FitEclipse(aPoints, fitCurve))
	{
		return 2;
	}
	//else if (FitBSpline(aPoints, fitCurve))
	//{
	//	return -1;
	//}
	else
	{
		cout << "Failed" << endl;
		return -1;
	}
}

inline bool SheetFlattenFitting::FitLine(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve)
{
	LineFitting lineFitting(aPoints);
	if (lineFitting.Perform())
	{
		gp_Pnt2d p1, p2;
		lineFitting.GetLineParameter(p1, p2);

		// 假设线段位于 XY 平面，Z = 0
		gp_Pnt p1_3d(p1.X(), p1.Y(), 0.0);  // 将 2D 点提升到 3D 空间
		gp_Pnt p2_3d(p2.X(), p2.Y(), 0.0);  // 将 2D 点提升到 3D 空间

		// 使用 BRepBuilderAPI_MakeEdge 创建 3D 线段
		BRepBuilderAPI_MakeEdge makeEdge = BRepBuilderAPI_MakeEdge(p1_3d, p2_3d);



		//BRepBuilderAPI_MakeEdge2d makeEdge = BRepBuilderAPI_MakeEdge2d(p1, p2);
		fitCurve.curveType = LineCurve;
		fitCurve.edge = makeEdge.Edge();

		fitCurve.line = make_tuple(p1, p2);

		cout << "Create Line." << endl;

		return true;
	}
	else
	{
		return false;
	}
}
inline bool SheetFlattenFitting::FitCircle(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve)
{
	if (aPoints->Size() < 3)
	{
		return false;
	}

	CircleFitting circleFitting(aPoints);
	if (circleFitting.Perform())
	{
		gp_Pnt2d center, p1, p2;
		double R;
		circleFitting.GetCircleParameter(center, R, p1, p2);


		double vector1X = p1.X() - center.X();
		double vector1Y = p1.Y() - center.Y();
		double vector2X = p2.X() - center.X();
		double vector2Y = p2.Y() - center.Y();

		// 计算角度
		double startAngle = atan2(vector1Y, vector1X); // p1的角度
		double endAngle = atan2(vector2Y, vector2X);




		gp_Dir2d theV(p1.X() - center.X(), p1.Y() - center.Y());
		
		gp_Ax2d theXAxis(center, theV);
		gp_Circ2d L(theXAxis, R);

		double U1, U2;

		gp_Dir2d theV2(p2.X() - center.X(), p2.Y() - center.Y());

		double angle = theV2.Angle(theV);

		angle = fabs(angle);

		Handle(Geom2d_Circle) C = new Geom2d_Circle(L);
		Handle(Geom2d_Curve) gemoCurveTrim = new Geom2d_TrimmedCurve(C, 0, angle);

		if (!IsPointOnCurve2d(aPoints->Value(2), gemoCurveTrim))
		{
			/*U1 = -angle;
			U2 = 0;*/
			U1 = endAngle;
			U2 = startAngle;
		}
		else
		{
			/*U1 = 0;
			U2 = angle;*/
			U1 = startAngle;
			U2 = endAngle;
			
		}


		// Create 3D circle from the 2D circle data
		gp_Pnt center3d(center.X(), center.Y(), 0.0);  // Assume the circle lies in the XY plane (Z=0)
		gp_Dir normal(0.0, 0.0, 1.0);  // Normal vector along the Z-axis

		// Create the 3D circle
		gp_Ax2 axis(center3d, normal); // Circle axis in 3D space
		gp_Circ circle3d(axis, R);     // 3D circle

		// Create 3D edge using the 3D circle
		BRepBuilderAPI_MakeEdge makeEdge3d(circle3d, U1, U2);

		//fitCurve.curveType = CircleCurve;
		//fitCurve.edge = makeEdge3d.Edge();  // Set the 3D edge

		//BRepBuilderAPI_MakeEdge2d makeEdge = BRepBuilderAPI_MakeEdge2d(L, U1, U2);		

		fitCurve.curveType = CircleCurve;
		fitCurve.edge = makeEdge3d.Edge();

		fitCurve.circle = make_tuple(center, R, U1, U2);

		cout << "Create Circle." << endl;

		return true;
	}
	
	return false;
	
}
inline bool SheetFlattenFitting::FitEclipse(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve)
{
	if (aPoints->Size() < 6)
	{
		return false;
	}

	EclipseFitting eclipseFitting(aPoints);

	if (eclipseFitting.Perform())
	{
		gp_Pnt2d center;
		double a, b, theta;
		gp_Pnt2d p1, p2;
		eclipseFitting.GetEclipseParameter(center, a, b, theta, p1, p2);

		// gp_Elips2d(const gp_Ax2d & theMajorAxis, const Standard_Real theMajorRadius, const Standard_Real theMinorRadius, const Standard_Boolean theIsSense = Standard_True)
		// gp_Ax2d (const gp_Pnt2d& theP, const gp_Dir2d& theV)
		//gp_Dir2d theV;
		//theV.Rotate(theta);
		//gp_Ax2d theXAxis(center, theV);
		//gp_Elips2d L(theXAxis, a, b);
		//BRepBuilderAPI_MakeEdge2d makeEdge = BRepBuilderAPI_MakeEdge2d(L, p1, p2);

		gp_Dir2d theV(p1.X() - center.X(), p1.Y() - center.Y());
		gp_Ax2d theXAxis(center, theV);
		gp_Elips2d L(theXAxis, a, b);

		double U1, U2;

		gp_Dir2d theV2(p2.X() - center.X(), p2.Y() - center.Y());

		double angle = theV2.Angle(theV);

		angle = fabs(angle);

		Handle(Geom2d_Ellipse) C = new Geom2d_Ellipse(L);
		Handle(Geom2d_Curve) gemoCurveTrim = new Geom2d_TrimmedCurve(C, 0, angle);

		if (!IsPointOnCurve2d(aPoints->Value(2), gemoCurveTrim))
		{
			U1 = -angle;
			U2 = 0;
		}
		else
		{
			U1 = 0;
			U2 = angle;
		}

		BRepBuilderAPI_MakeEdge2d makeEdge = BRepBuilderAPI_MakeEdge2d(L, U1, U2);

		fitCurve.curveType = EclipseCurve;
		fitCurve.edge = makeEdge.Edge();

		double firstPara, secondPara;
		Handle(Geom_Curve) gemoCurve = BRep_Tool::Curve(fitCurve.edge, firstPara, secondPara);

		fitCurve.eclipse = make_tuple(center, a, b, theta, firstPara, secondPara);

		cout << "Create Eclipse." << endl;

		return true;
	}

	return false;
}
inline bool SheetFlattenFitting::FitBSpline(const Handle(TColgp_HArray1OfPnt2d)& aPoints, FitCurve& fitCurve)
{
	//Geom2dAPI_Interpolate aInterpolater(aPoints, Standard_False, Precision::Approximation());
	//aInterpolater.Perform();
	//Handle(Geom2d_BSplineCurve) aBSplineCurve;
	//aBSplineCurve = aInterpolater.Curve();

	// BRepBuilderAPI_MakeEdge2d makeEdge = BRepBuilderAPI_MakeEdge2d(aBSplineCurve);

	//fitCurve.curveType = BSplineCurve;
	//fitCurve.edge = makeEdge.Edge();

	//cout << "Create BSpline." << endl;

	return true;
}

inline bool SheetFlattenFitting::IsPointOnCurve2d(const gp_Pnt2d& pt, const Handle(Geom2d_Curve)& curve)
{
	Geom2dAPI_ProjectPointOnCurve projectTooler(pt, curve);

	double dis;

	try
	{
		dis = projectTooler.LowerDistance();
	}
	catch (StdFail_NotDone)
	{
		cout << "StdFail_NotDone" << endl;
		return false;
	}
	
	
	//gp_Pnt2d npt = projectTooler.NearestPoint();
	//npt.DumpJson(cout);
	//cout << endl;
	//cout << dis << endl;

	if (dis > 1e-3)
	{
		return false;
	}
	else
	{
		return true;
	}
}