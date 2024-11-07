/*HEAD BodyDes_OnestepUnformSolver HXX AUTOMOTIVE_CAE */
/*==================================================================================================

                    Copyright 2006 UGS Corp.
                      All rights reserved

====================================================================================================
    File description:

    Data definitions of Onestep unform solver

====================================================================================================
Date         Name                    Description of Change
14-Feb-2006  Xianghui Zhang          Written
20-Apr-2006  Jerry Hu                Added headcard & Fixed QAZ errors
20-Jun-2006  Jerry Hu                Added namespaces
26-Jun-2006  liao haiwen             Add the data member min_node_id and nodes_id_on_edges
29-Jun-2006  Xianghui Zhang          Add OnestepUnform_GetThickness, OnestepUnform_GetStrain and OnestepUnform_GetStress.
                                     to OnestepUnformMesh_t
30-Jun-2006  Jerry Hu                Removed CLINKAGE
30-Jun-2006  Xiangkui Zhang          Move solver function declare from namespace UGS::BodyDes to namespace KMAS.
18-Jul-2006  Xiangkui Zhang          Simplify the interfaces of solver.
20-Jul-2006  Jerry Hu                Use double to keep data type consistent
24-Jul-2006  Jerry Hu                Removed macros and changed MESH_BAD to MESH_POOR_QUALITY
26-Jul-2006  Xiangkui Zhang          Added more API
04-Aug-2006  Xiangkui Zhang          Added OnestepUnform_GetSpringbackShape
16-Aug-2006  Jerry Hu                Added NX standard export
18-Oct-2006  Jerry Hu                Added SetRefNode and GetRefNode
15-Nov-2006  Xiangkui Zhang          Added SPRINGBACK_POST_ERROR
13-Mar-2007  Jing-qi Wang            Added struct TOnestepSolverSettings
14-Mar-2007  Xiangkui Zhang          Added OnestepUnform_SetSolverSetting and OnestepUnform_GetSolverSetting, the setting
                                     information will be supported by prf file.
06-Jun-2007  Xiangkui Zhang          Added OnestepUnform_SetExtraNormalInfor
25-Jun-2007  Hu Si-bo                Added OnestepUnform_SaveSpringbackResultFile
25-Oct-2007  Xiangkui Zhang          Added OnestepUnform_SolverEx and OnestepUnform_SetMeshEx.
26-Oct-2007  Xiangkui Zhang          Added OnestepUnform_CheckMeshOverlap, OnestepUnform_CheckMeshQuality and
                                     OnestepUnform_CheckThickness.
12-May-2009  Ping Zhou               Added OnestepUnform_ImportFemMesh,OnestepUnform_ImportProjectDirection,
12-May-2009  Ping Zhou               Added OnestepUnform_SetBlankholderMesh,OnestepUnform_SetBlankholderForce,
03-Jun-2009  Xiangkui Zhang          Added OnestepUnform_GetBorderLoops which is used to get a set of loops of border nodes.
14-Nov-2009  Jianwei Zhang           Added m_solverJoinOutputCurves setting
15-Nov-2010  Xiangkui Zhang          Added OnestepUnform_GetTopSurfaceStress,OnestepUnform_GetTopSurfaceStrain,
                                     OnestepUnform_GetBottomSurfaceStress,OnestepUnform_GetBottomSurfaceStrain
02-Apr-2011  Xiangkui Zhang          Added "extern C_LINKAGE" option for all interface functions of libonestep
05-May-2011  Xiangkui Zhang          Keep C++ interfaces version, and added new "extern C_LINKAGE" interfaces, such as:
                                     New: extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformConstructor(TOnestepSolverType type);
                                     Old: ONESTEPEXPORT bool OnestepUnform_Constructor(TOnestepSolverType type);
24-May-2011  Jerry Hu                Removed the unidef header file from hxx file
10-Jul-2012  Xiangkui Zhang          Support dynamic loading API
15-Jul-2012  Xiangkui Zhang          Added OnestepUnform_ReverseUnformSide which is used to support toggle button 
                                     "Reverse Unform Side", corresponding C_LINKAGE function name is 
                                     OnestepUnformReverseUnformSide.
06-Dec-2012  Sahil Jasrotia          ARCH9321: Moved this file from libautomotive library
23-Dec-2014  Sachin Mahalle          ARCH11324: Removed usage of IPLIB macro
                                     hard points by coordinates note: Only Support Triangle Meshes
15-Feb-2015  Xiangkui Zhang          Added OnestepUnform_SetMeshesEx2 which supports automatic generation of 
                                     hard points by coordinates note: Only Support Triangle Meshes
09-Jul-2015  Xiangkui Zhang          Added OnestepUnform_SetMeshesEx3: Import group info On the basis of 
                                     OnestepUnform_SetMeshesEx2
                                     Notify: Only Support Triangle Meshes
06-Aug-2016  Xiangkui Zhang          Added Trim Line Support Functions: 
                                     OnestepUnformTrimLineSolve
                                     OnestepUnform_TrimLineSolve
                                     OnestepUnformCheckTrimLineSolveParameters
                                     OnestepUnformCheck_TrimLineSolveParameters
08-Mar-2017  Lynette Xu              Add trim line thickness point and thickness direction
09-Sep-2018  Xiangkui Zhang          Add Bulking Coefficient Support Functions:
                                     OnestepUnform_SetBulkingCoefficient
                                     OnestepUnformSetBulkingCoefficient
                                     Notify: Bulking Coefficient is only applicable to stamping parts excluding overlapping areas,
                                     and only whole unfolding without target regions is supported,
                                     the parameter "bulking_coefficient" value must be between 0.0 and 1.0, 
                                     otherwise false will be returned.
$HISTORY$
==================================================================================================*/

#ifndef BodyDes_OnestepUnformSolver_HXX_INCLUDED
#define BodyDes_OnestepUnformSolver_HXX_INCLUDED

#ifdef BUILD_OUTSIDE_NX_ENV
#if defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#ifdef LIBONESTEP_EXPORTS
#define ONESTEPEXPORT __declspec(dllexport)
#else
#define ONESTEPEXPORT __declspec(dllimport)
#endif
#else
#define ONESTEPEXPORT
#endif
#else
#include <libonestep_exports.h>
#endif

#ifndef C_LINKAGE
#define C_LINKAGE "C"
#endif

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the Onestep_SOLVER_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// Onestep_SOLVER_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
//#ifdef DLL_EXPORTS
//#define ONESTEPEXPORT __declspec(dllexport)
//#else
//#define ONESTEPEXPORT __declspec(dllimport)
//#endif

#define SolverValidationError_InvalidSheetCount 1 
#define SolverValidationError_NoContactRegion 3 
#define SolverValidationError_NoTaregetRegion 4
#define SolverValidationError_NoContactPoints 5
#define SolverValidationError_InvalidMeshElements 6
#define SolverValidationError_SoverInternalUndefinedError 7
#define SolverValidationError_ToleranceIsSmall  20
#define SolverValidationError_OverlappedMeshElementsOnTargetRegion   21
#define SolverValidationError_GapsOnTargetRegion  22


namespace KMAS
{

    enum TOnestepSolverErrorType
    {
        SUCCESSFUL = 0,
        QUADRILATERAL_ERROR = -1,
        MESH_POOR_QUALITY = -2,
        SPRINGBACK_POST_ERROR = -3,
        FAIL_SENSE_LOCK = -4,
        LIMIT_NODE_ERROR = -5,
        CAC_PRE_CHECK_FAIL = -6,
        UNKNOW_ERROR = -100
    };

    enum TOnestepSolverType
    {
        WHOLE_UNFOLD = 0,
        INTERMEDIATE_UNFOLD = 1
    };

    struct TOnestepSolverMaterialProp
    {
        double e_data;
        double yield_stress_data;
        double poisson_rate_data;
        double density_data;
        double harding_exponent_data;      //"N" in class OnestepUnformSettings
        double initial_strain_data;
        double friction_data;
        double strength_coefficient_data;  //"K" in class OnestepUnfromSettings
        double r0_data;
        double r45_data;
        double r90_data;
    };

    struct TOnestepSolverBorderInfo
    {
        int id;
        double x;
        double y;
        double z;
    };

    struct TOnestepSolverSettings 
    {
        //define settings for material properties
        double        m_materialPropertyE;
        double        m_materialPropertyDensity;
        double        m_materialPropertyPoisson;
        double        m_materialPropertyYieldStress;
        double        m_materialPropertyN;
        double        m_materialPropertyInitialStrain;
        double        m_materialPropertyFriction;
        double        m_materialPropertyK;
        double        m_materialPropertyR0;
        double        m_materialPropertyR45;
        double        m_materialPropertyR90;

        //define settings for mesh
        int           m_meshElementType;
        bool          m_meshInferElementSize;
        double        m_meshElementSize;
        bool          m_meshSplitQuad;
        double        m_meshMaxWarp;
        double        m_meshMaxJacobian;
        bool          m_meshAttemptMapping;
        int           m_meshSizeVariation;
        bool          m_meshProcessFillet;
        double        m_meshSmallFeature;

        //define settings for solver
        double        m_solverHoldingForce;
        int           m_solverConvergencyLevel;
        int           m_solverMaxIterationSteps;
        bool          m_solverDoSpringbackCalcultaion;
        bool          m_solverSaveAnalysisResultsIntoFeature;
        bool          m_solverJoinOutputCurves;

        //define settings for report display
        bool       m_reportDisplayThickness;
        bool       m_reportDisplayStress;
        bool       m_reportDisplayStrain;
        bool       m_reportDisplaySpringback;
        bool       m_reportDisplayFlattenShape;
        bool       m_reportDisplayViewControl;
    };

    typedef void (*pfnKmasOnestepSolverCallback_t)(const int &step, const char * const msg);
    typedef void (*pfnKmasOnestepProjectCallback_t)(
        double *coords,         //I coordinate of nodes (in sequence)
        int nlen,               //I the number of nodes 
        unsigned long edge_tag, //I edge tag
        double advice_value,    //I advice value of extension
        double *n_coords        //O: projection coordinate on extension of target region
        );

    enum TOnestepSolverSurfaceType
    {
        INNER_SURFACE,
        MID_SURFACE,
        OUTER_SURFACE
    };

    // edited by zhou ping 2009.05.12
    // define onestep solver node
#ifndef _KMAS_ONESTEP_SOLVER_NODE
#define _KMAS_ONESTEP_SOLVER_NODE
    struct TOnestepSolverNode
    {
        double v[3];
        int    constraint_id;
    };
#endif
    // end define onestep solver node

    // define onestep solver element
#ifndef _KMAS_ONESTEP_SOLVER_ELEMENT
#define _KMAS_ONESTEP_SOLVER_ELEMENT
    struct TOnestepSolverElement
    {
        int n[4];
    };
#endif
    // end define onestep solver element

    // define drawbead element
#ifndef _KMAS_ONESTEP_SOLVER_DRAWBEAD_ELEMENT
#define _KMAS_ONESTEP_SOLVER_DRAWBEAD_ELEMENT
    struct TOnestepSolverDrawbeadElem
    {
        int     node1;          // node1 num in element
        int     node2;          // node2 num in element
        int     DBgroup;        // drawbead group number
        int     ElemId;         // element number
    };
#endif
    // end define drawbead element

    // define drawbead property
#ifndef _KMAS_ONESTEP_SOLVER_DRAWBEAD_PROP
#define _KMAS_ONESTEP_SOLVER_DRAWBEAD_PROP
    struct TOnestepSolverDrawbeadProp
    {
        double  t_tension;      // horizontal resistance of drawbead element
        double  n_tension;      // vertical resistance of drawbead element
        double  db_friction;    // friction coefficient
    };
#endif
    // end define drawbead property

    // define blankholder property
#ifndef _KMAS_ONESTEP_SOLVER_BLANKHOLDER_PROP
#define _KMAS_ONESTEP_SOLVER_BLANKHOLDER_PROP
    struct TOnestepSolverBlankholderProp
    {
        double  force;          // force on holder blank
        double  pressure;       // pressure on holder blank
        double  bh_friction;    // friction coefficient
    };
#endif
    // end define blankholder property

    // define boundary force
#ifndef _KMAS_ONESTEP_SOLVER_BOUNDARY_FORCE
#define _KMAS_ONESTEP_SOLVER_BOUNDARY_FORCE
    struct TOnestepSolverBoundaryElem 
    {
        int     node1;      //node1 num of boundary element
        int     node2;      //node2 num of boundary element
        int     DBgroup;    //boundary group number
        int     ElemId;     //boundary element number
    };
#endif
    // end define boundary force

    // define boundary property
#ifndef _KMAS_ONESTEP_SOLVER_BOUNDARY_PROP
#define _KMAS_ONESTEP_SOLVER_BOUNDARY_PROP
    struct TOnestepSolverBoundaryProp
    {
        double      force;          //force on boundary
        double      tension;        //resistance on boundary element
        double      bf_friction;    //friction coefficient
    };
#endif
    // end define boundary property
    // end edited by zhou ping 2009.05.12

/*07/13/2016 Xiangkui Zhang, add, start*/
#ifndef  _KMAS_ONESTEP_SOLVER_TRIMLINE_PARAMETERS
#define _KMAS_ONESTEP_SOLVER_TRIMLINE_PARAMETERS
    struct TOnestepSolverTrimLineParameters 
    {
        //draw direction-->thickness direction
        double dx;
        double dy;
        double dz;
        //thickness of part
        double thickness;

        //surface type: middle surface, outer surface and innner surface;
        //discarded parameter, keep it for backward compatibility
        TOnestepSolverSurfaceType surface_type;

        //thickness direction node id
        int td_nid;

        //material prop
        TOnestepSolverMaterialProp material;
        double tol;
        double angle;
        double scale;
    public:
        TOnestepSolverTrimLineParameters()
        {
            dx = 0.0;
            dy = 0.0;
            dz = 1.0;

            thickness = 1.0;
            surface_type = MID_SURFACE;

            tol = 0.0;
            angle = 25;
            scale = 1.0;
            td_nid = -1;
            material.e_data = 0.0;
            material.yield_stress_data = 0.0;
            material.poisson_rate_data = 0.0;
            material.density_data = 0.0;
            material.harding_exponent_data = 0.0;
            material.initial_strain_data = 0.0;
            material.friction_data = 0.0;
            material.strength_coefficient_data = 0.0;
            material.r0_data = 0.0;
            material.r45_data = 0.0;
            material.r90_data = 0.0;
        }
    };
#endif
/*07/13/2016 Xiangkui Zhang, add, end*/

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformConstructor(TOnestepSolverType type);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformDestructor();

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetSolverType(TOnestepSolverType &type);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetDrawDirection(const double tdx, const double tdy, const double tdz);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetDrawDirection(double &tdx, double &tdy, double &tdz);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMaterialProp(const TOnestepSolverMaterialProp &prop);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetMaterialProp(TOnestepSolverMaterialProp &prop);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetSolverSetting(const TOnestepSolverSettings &val);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetSolverSetting(TOnestepSolverSettings &val);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBlankThickness(const double &thickness);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetBlankThickness(double &thickness);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetKmasFullFilename(const char *fname);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetKmasFullFilename(char *fname);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetRefElement(const int &eid);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetRefElement(int &eid);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetRefNode(const int &eid);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetRefNode(int &eid);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetRefBoundary(const int &master_nid, const int &slave_nid);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetRefBoundary(int &master_nid, int &slave_nid);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetSurfaceType(const TOnestepSolverSurfaceType &surface_type);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetSurfaceType(TOnestepSolverSurfaceType &surface_type);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformRegisterCallback(pfnKmasOnestepSolverCallback_t cb);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMinNodeID(int min_id);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetMinNodeID(int &min_id);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetNodeIDsOnFreeEdge(int *nids, int *index, int ecount);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetNodeIDsOnFreeEdge(int *&nids, int *&index, int &ecount);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMeshes(TOnestepSolverNode *nodes, const int &nlen, TOnestepSolverElement *elements, const int &elen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetMeshes(TOnestepSolverNode *&nodes, int &nlen, TOnestepSolverElement *&elements, int &elen);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformFillHole( double *coords, int &nlen, 
        int *elements, int &elen, double *&coords_res, int *&elements_res);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMeshesExKeepHole( double *coords, int nlen, 
        int *elements, int elen,
        int mode,// Whole unfold: 0 - there isn't target surface.
        //               1 - there exist target surface.
        // Intermediate unfold: 0 - common-edge.
        //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        int *unform_cIds, int uclen, 
        int *target_cIds, int tclen );

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMeshesEx( double *coords, int nlen, 
        int *elements, int elen,
        int mode,// Whole unfold: 0 - there isn't target surface.
        //               1 - there exist target surface.
        // Intermediate unfold: 0 - common-edge.
        //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        int *unform_cIds, int uclen, 
        int *target_cIds, int tclen );
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMeshesEx2(
        double *coords, int nlen, 
        int *elements, int elen,
        int mode,   // Whole unfold: 0 - there isn't target surface.
                    //                        1 - there exist target surface.
                    // Intermediate unfold: 0 - common-edge.
                    //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        double *unform_cPts, int uclen, 
        double *target_cPts, int tclen ); //Only Support Triangle Meshes
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMeshesEx3( double *coords, int nlen, 
        int *elements, int elen,
        int mode,// Whole unfold: 0 - there isn't target surface.
        //                        1 - there exist target surface.
        // Intermediate unfold: 0 - common-edge.
        //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        double *unform_cPts, int uclen, 
        double *target_cPts, int tclen,
        int *grp_info,  //Sample: 2 - two groups;
                        //        123(unform cids number) 43(target cids number) - frist group
                        //        32(unform cids number) 23(target cids number) - second group
                        //        grp_info: 2,123,43,32,23; grp_info array length is (grp_info[0] * 2 + 1)
                        //        in this time, (uclen == 123 + 32) and (tclen == 43 + 23)
        int is_first_grp_p2p);//Only Support Triangle Meshes
    /* 2011-11-21 Liuweijie Add for process curve along curve Start */
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetCACConstraints(
        int *CAC_unform_cIds, int uclen,
        int *CAC_target_cIds, int tclen);
    /* 2011-11-21 Liuweijie Add for process curve along curve End */
    /* 2012-02-20 Liuweijie Add for process curve along curve Start */
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetCACConstraintsFormal(
        int *CAC_unform_cIds, int uclen, int *CAC_UclenEach, int ucelen,
        int *CAC_target_cIds, int tclen, int *CAC_TclenEach, int tcelen);
    /* 2012-02-20 Liuweijie Add for process curve along curve End */

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSolve(TOnestepSolverErrorType &ret, const bool &s_flag);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSolveEx(TOnestepSolverErrorType &ret, const bool &s_flag);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetBlankShape(double *nodes, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetThickness(double *thickness, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetStress(double *stresses, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetStrain(double *strains, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetSpringbackShape(double *nodes, const int &nlen);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetTopSurfaceStress(double *stresses, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetTopSurfaceStrain(double *strains, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetBottomSurfaceStress(double *stresses, const int &nlen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetBottomSurfaceStrain(double *strains, const int &nlen);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetBorders(double **coords, int **bids, int *bids_len, int **index, int *nloops);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformLoadNastranFile(const char *fname);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSaveNastranFile(const char *fname, bool is_origin = true);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformIsResultExist();
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformIsSpringbackResultExist();
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformLoadResultFile(const char *fname, int &nlen, int &elen);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSaveResultFile(const char *fname);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetSpringbackRefNode(const int ref_nid1, const int ref_nid2, const int ref_nid3);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetExtraNormalInfor(int node_normalnum,int *node_normalID,double *node_normal);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSaveSpringbackResult(const char *fname);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformLoadGroupInfoFile(const char *fname);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformTwoStepTest();

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformCheckMeshOverlap(bool &is_passed);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformCheckMeshQuality(bool &is_passed);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformCheckThickness(bool &is_passed);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetSolverMemorySize(const int mb_size);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformRegisterProjectCallback(pfnKmasOnestepProjectCallback_t cb);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBorderInfo(
        unsigned long *tags, //I the tags of border (in sequence)
        int tags_num,        //I the number of edge tags.
        int *nids,           //I the nodes on the corresponding edge.(in sequence)
        int *group_info      //I the group info of the edge nodes.(in sequence), and its length is tags_num,
        //  its value is the nodes number of the corresponding edge.
        );
    // edited by zhou ping 2009.05.12
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformImportFemMesh(TOnestepSolverNode *nodes, const int &nlen, TOnestepSolverElement *elements, const int &elen, const double &max_length);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformImportProjectDirection(double v[], const int len = 3);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformImportDrawbeadCurve(TOnestepSolverNode *drawbead_point, int *group_num, int num_group);
    extern C_LINKAGE ONESTEPEXPORT int  OnestepUnformGetDrawbeadMesh(TOnestepSolverNode *&drawbeadnode , TOnestepSolverDrawbeadElem *&drawbeadelement, int &nodenum, int &elemnum);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBlankholderMesh(int *blankholderElem, const int elemnum);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBlankholderForce(const TOnestepSolverBlankholderProp blankholder_prop);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetDrawbeadMesh(TOnestepSolverNode *drawbeadnode, TOnestepSolverDrawbeadElem *drawbeadelement, const int nodenum, const int elemnum);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetDrawbeadForce(TOnestepSolverDrawbeadProp *drawbeadforce, const int drawbeadGroupNum);

    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBoundaryMesh(TOnestepSolverNode *boundarynode, TOnestepSolverBoundaryElem *boundaryelement, const int nodenum, const int elemnum);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBoundaryForce(TOnestepSolverBoundaryProp *boundaryforce, const int boundaryGroupNum);
    // end edited by zhou ping 2009.05.12

    /*06/03/2009, Xiangkui Zhang, add, start*/
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetBorderLoops(int *&nids, int *&loop_len, int &loop_num);
    /*06/03/2009, Xiangkui Zhang, add, end*/

    /*07/15/2012 Xiangkui Zhang, add, start*/
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformReverseUnformSide(bool bReverse); // bReverse default value is false.
    /*07/15/2012 Xiangkui Zhang, add, end*/

    /*03/16/1016 Xiangkui Zhang, add, start*/
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetDiamondElementConstrain(int *de_cids, const int de_len);

    //Implicit condition: before this function is invoked, unform_bend_partid should be clone from unform_flatten_partid.
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformMappingMesh(unsigned long long ulModel, int target_bend_partid, int target_flattend_partid, int unform_flatten_partid,int unform_bend_partid);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformMappingMeshEx(unsigned long long ulModel, int target_bend_partid, int target_flattend_partid, int *area_eids, int area_elen, double *area_points);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetMainArea(double *mpoints, int nlen, int *meids, int elen);
    /*03/16/1016 Xiangkui Zhang, add, end*/

    /*07/13/2016 Xiangkui Zhang, add, start*/
     /*Return Flag: 
     * 0-Successful; 
     * Error: Solver will be aborted.
     * 1-Part number is error: must have and only two connected mesh parts
     * 2-Output Log;
     * 3-share node ids array size is zero; snlen == 0 or share_nids == NULL;
     * 4-target region's element ids array size is zero. telen == 0 or target_eids == NULL;
     * 5-point coordinates array size is zero. vnlen == 0 or vpts == NULL;
     * 6-elements array size is zero. velen == 0 or velements == NULL;
     * 7-unknown error
    */
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformTrimLineSolve(
        double *pts/*IO*/, int nlen/*I*/,                                                          //points coordinates array, result will be returned by this parameter; this array's size is neln * 3
        int *elements/*I*/, int elen/*I*/,                                                          //elements array: n1,n2,n3,n4... and n3 == n4 if element is triangle; this array's size is elen * 4
        int *target_eids/*I*/, int telen/*I*/,                                                      //target region's element ids array; this array's size is telen
        int *share_nids/*I*/, int snlen/*I*/,                                                      //share node ids array between unform region and target region (share rule: distance < tol); this array's size is snlen
        const KMAS::TOnestepSolverTrimLineParameters &para/*I*/,           // parameters: draw direction/thickness/surface type/material,
        double *ppts_of_share_nids/*I*/,                                                        // the projection points array form share node to target regions.  this array's size is snlen * 3
        int &nRetCode/*O*/                                  
        );
    /*Return Flag: 
     * 0-Successful; 
     * Error: Solver will be aborted.
     * 1-Part number is error: must have and only two connected mesh parts
     * 3-share node ids array size is zero; snlen == 0 or share_nids == NULL;
     * 4-target region's element ids array size is zero. telen == 0 or target_eids == NULL;
     * 5-point coordinates array size is zero. vnlen == 0 or vpts == NULL;
     * 6-elements array size is zero. velen == 0 or velements == NULL;
     * 7-unknown error
     * Warning: Solver may continue to execute
     * 20-share element ids number < unform element ids number * 40%, it seem that tol is too small, the result of solver may be bad.
     * 21-target meshes have overlap elements with current draw direction, the result of solver may be bad
     * 22-there are gaps in target region faces , the result of solver may be bad.
    */
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformCheckTrimLineSolveParameters(
        double *pts/*IO*/, int nlen/*I*/,                                                          //points coordinates array, result will be returned by this parameter; this array's size is neln * 3
        int *elements/*I*/, int elen/*I*/,                                                          //elements array: n1,n2,n3,n4... and n3 == n4 if element is triangle; this array's size is elen * 4
        int *target_eids/*I*/, int telen/*I*/,                                                      //target region's element ids array; this array's size is telen
        int *share_nids/*I*/, int snlen/*I*/,                                                      //share node ids array between unform region and target region (share rule: distance < tol); this array's size is sneln
        const KMAS::TOnestepSolverTrimLineParameters &para/*I*/,           // parameters: draw direction/thickness/surface type/material,
        double *ppts_of_share_nids/*I*/,                                                        // the projection points array form share node to target regions.  this array's size is sneln * 3
        int &nRetCode/*O*/);
    /*07/13/2016 Xiangkui Zhang, add, end*/

    /*08/13/2016 Xiangkui Zhang, add, start*/
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetShareElementIDs(
        int *&share_eids/*O*/,
        int &selen/*O*/);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformGetShareNodeIDs(
        int *&share_nids/*O*/,
        int &snlen/*O*/);
    /*08/13/2016 Xiangkui Zhang, add, end*/
    /*09/19/2018 Xiangkui Zhang, Add, Start*/
    //Notify: This API is only applicable to stamping parts excluding overlapping areas.
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformSetBulkingCoefficient(double bulking_coefficient);
    /*09/19/2018 Xiangkui Zhang, Add, End*/

    ONESTEPEXPORT bool OnestepUnform_Constructor(TOnestepSolverType type);
    ONESTEPEXPORT bool OnestepUnform_Destructor();

    ONESTEPEXPORT bool OnestepUnform_GetSolverType(TOnestepSolverType &type);

    ONESTEPEXPORT bool OnestepUnform_SetDrawDirection(const double tdx, const double tdy, const double tdz);
    ONESTEPEXPORT bool OnestepUnform_GetDrawDirection(double &tdx, double &tdy, double &tdz);

    ONESTEPEXPORT bool OnestepUnform_SetMaterialProp(const TOnestepSolverMaterialProp &prop);
    ONESTEPEXPORT bool OnestepUnform_GetMaterialProp(TOnestepSolverMaterialProp &prop);

    ONESTEPEXPORT bool OnestepUnform_SetSolverSetting(const TOnestepSolverSettings &val);
    ONESTEPEXPORT bool OnestepUnform_GetSolverSetting(TOnestepSolverSettings &val);

    ONESTEPEXPORT bool OnestepUnform_SetBlankThickness(const double &thickness);
    ONESTEPEXPORT bool OnestepUnform_GetBlankThickness(double &thickness);

    ONESTEPEXPORT bool OnestepUnform_SetKmasFullFilename(const char *fname);
    ONESTEPEXPORT bool OnestepUnform_GetKmasFullFilename(char *fname);

    ONESTEPEXPORT bool OnestepUnform_SetRefElement(const int &eid);
    ONESTEPEXPORT bool OnestepUnform_GetRefElement(int &eid);

    ONESTEPEXPORT bool OnestepUnform_SetRefNode(const int &eid);
    ONESTEPEXPORT bool OnestepUnform_GetRefNode(int &eid);

    ONESTEPEXPORT bool OnestepUnform_SetRefBoundary(const int &master_nid, const int &slave_nid);
    ONESTEPEXPORT bool OnestepUnform_GetRefBoundary(int &master_nid, int &slave_nid);

    ONESTEPEXPORT bool OnestepUnform_SetSurfaceType(const TOnestepSolverSurfaceType &surface_type);
    ONESTEPEXPORT bool OnestepUnform_GetSurfaceType(TOnestepSolverSurfaceType &surface_type);

    ONESTEPEXPORT bool OnestepUnform_RegisterCallback(pfnKmasOnestepSolverCallback_t cb);

    ONESTEPEXPORT bool OnestepUnform_SetMinNodeID(int min_id);
    ONESTEPEXPORT bool OnestepUnform_GetMinNodeID(int &min_id);
    ONESTEPEXPORT bool OnestepUnform_SetNodeIDsOnFreeEdge(int *nids, int *index, int ecount);
    ONESTEPEXPORT bool OnestepUnform_GetNodeIDsOnFreeEdge(int *&nids, int *&index, int &ecount);
    ONESTEPEXPORT bool OnestepUnform_SetMeshes(TOnestepSolverNode *nodes, const int &nlen, TOnestepSolverElement *elements, const int &elen);
    ONESTEPEXPORT bool OnestepUnform_GetMeshes(TOnestepSolverNode *&nodes, int &nlen, TOnestepSolverElement *&elements, int &elen);

    ONESTEPEXPORT bool OnestepUnform_FillHole( double *coords, int &nlen, 
        int *elements, int &elen, double *&coords_res, int *&elements_res);

    ONESTEPEXPORT bool OnestepUnform_SetMeshesEx( double *coords, int nlen, 
        int *elements, int elen,
        int mode,// Whole unfold: 0 - there isn't target surface.
        //               1 - there exist target surface.
        // Intermediate unfold: 0 - common-edge.
        //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        int *unform_cIds, int uclen, 
        int *target_cIds, int tclen );
    ONESTEPEXPORT bool OnestepUnform_SetMeshesEx3(double *coords, int nlen, 
        int *elements, int elen,
        int mode,// Whole unfold: 0 - there isn't target surface.
        //                        1 - there exist target surface.
        // Intermediate unfold: 0 - common-edge.
        //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        double *unform_cPts, int uclen, 
        double *target_cPts, int tclen,
        int *grp_info,  //Sample: 2 - two groups;
                        //        123(unform cids number) 43(target cids number) - frist group
                        //        32(unform cids number) 23(target cids number) - second group
                        //        grp_info: 2,123,43,32,23; grp_info array length is (grp_info[0] * 2 + 1)
                        //        in this time, (uclen == 123 + 32) and (tclen == 43 + 23)
        int is_first_grp_p2p); //Only Support Triangle Meshes
    ONESTEPEXPORT bool OnestepUnform_SetMeshesEx2( 
        double *coords, int nlen, 
        int *elements, int elen,
        int mode,   // Whole unfold: 0 - there isn't target surface.
                    //                        1 - there exist target surface.
                    // Intermediate unfold: 0 - common-edge.
                    //                      1 - uncommon-edge.
        int *target_eIds, int telen,
        double *unform_cPts, int uclen, 
        double *target_cPts, int tclen ); //Only Support Triangle Meshes

     /* 2011-11-21 Liuweijie Add for process curve along curve Start */
     ONESTEPEXPORT bool OnestepUnform_SetCACConstraints(
         int *CAC_unform_cIds, int uclen,
         int *CAC_target_cIds, int tclen);
     /* 2011-11-21 Liuweijie Add for process curve along curve End */
     /* 2012-02-20 Liuweijie Add for process curve along curve Start */
     ONESTEPEXPORT bool OnestepUnform_SetCACConstraintsFormal(
         int *CAC_unform_cIds, int uclen, int *CAC_UclenEach, int ucelen,
         int *CAC_target_cIds, int tclen, int *CAC_TclenEach, int tcelen);
     /* 2012-02-20 Liuweijie Add for process curve along curve End */

    ONESTEPEXPORT bool OnestepUnform_Solve(TOnestepSolverErrorType &ret, const bool &s_flag);
    ONESTEPEXPORT bool OnestepUnform_SolveEx(TOnestepSolverErrorType &ret, const bool &s_flag);

    ONESTEPEXPORT bool OnestepUnform_GetBlankShape(double *nodes, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetThickness(double *thickness, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetStress(double *stresses, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetStrain(double *strains, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetSpringbackShape(double *nodes, const int &nlen);

    ONESTEPEXPORT bool OnestepUnform_GetTopSurfaceStress(double *stresses, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetTopSurfaceStrain(double *strains, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetBottomSurfaceStress(double *stresses, const int &nlen);
    ONESTEPEXPORT bool OnestepUnform_GetBottomSurfaceStrain(double *strains, const int &nlen);

    ONESTEPEXPORT bool OnestepUnform_GetBorders(double **coords, int **bids, int *bids_len, int **index, int *nloops);

    ONESTEPEXPORT bool OnestepUnform_LoadNastranFile(const char *fname);
    ONESTEPEXPORT bool OnestepUnform_SaveNastranFile(const char *fname, bool is_origin = true);

    ONESTEPEXPORT bool OnestepUnform_IsResultExist();
    ONESTEPEXPORT bool OnestepUnform_IsSpringbackResultExist();
    ONESTEPEXPORT bool OnestepUnform_LoadResultFile(const char *fname, int &nlen, int &elen);
    ONESTEPEXPORT bool OnestepUnform_SaveResultFile(const char *fname);

    ONESTEPEXPORT bool OnestepUnform_SetSpringbackRefNode(const int ref_nid1, const int ref_nid2, const int ref_nid3);
    ONESTEPEXPORT bool OnestepUnform_SetExtraNormalInfor(int node_normalnum,int *node_normalID,double *node_normal);
    ONESTEPEXPORT bool OnestepUnform_SaveSpringbackResult(const char *fname);
    ONESTEPEXPORT bool OnestepUnform_LoadGroupInfoFile(const char *fname);
    ONESTEPEXPORT bool OnestepUnform_TwoStepTest();

    ONESTEPEXPORT bool OnestepUnform_CheckMeshOverlap(bool &is_passed);
    ONESTEPEXPORT bool OnestepUnform_CheckMeshQuality(bool &is_passed);
    ONESTEPEXPORT bool OnestepUnform_CheckThickness(bool &is_passed);

    ONESTEPEXPORT bool OnestepUnform_SetSolverMemorySize(const int mb_size);

    ONESTEPEXPORT bool OnestepUnform_RegisterProjectCallback(pfnKmasOnestepProjectCallback_t cb);
    ONESTEPEXPORT bool OnestepUnform_SetBorderInfo(
        unsigned long *tags, //I the tags of border (in sequence)
        int tags_num,        //I the number of edge tags.
        int *nids,           //I the nodes on the corresponding edge.(in sequence)
        int *group_info      //I the group info of the edge nodes.(in sequence), and its length is tags_num,
        //  its value is the nodes number of the corresponding edge.
        );
    // edited by zhou ping 2009.05.12
    ONESTEPEXPORT bool OnestepUnform_ImportFemMesh(TOnestepSolverNode *nodes, const int &nlen, TOnestepSolverElement *elements, const int &elen, const double &max_length);
    ONESTEPEXPORT bool OnestepUnform_ImportProjectDirection(double v[], const int len = 3);
    ONESTEPEXPORT bool OnestepUnform_ImportDrawbeadCurve(TOnestepSolverNode *drawbead_point, int *group_num, int num_group);
    ONESTEPEXPORT int  OnestepUnform_GetDrawbeadMesh(TOnestepSolverNode *&drawbeadnode , TOnestepSolverDrawbeadElem *&drawbeadelement, int &nodenum, int &elemnum);

    ONESTEPEXPORT bool OnestepUnform_SetBlankholderMesh(int *blankholderElem, const int elemnum);
    ONESTEPEXPORT bool OnestepUnform_SetBlankholderForce(const TOnestepSolverBlankholderProp blankholder_prop);

    ONESTEPEXPORT bool OnestepUnform_SetDrawbeadMesh(TOnestepSolverNode *drawbeadnode, TOnestepSolverDrawbeadElem *drawbeadelement, const int nodenum, const int elemnum);
    ONESTEPEXPORT bool OnestepUnform_SetDrawbeadForce(TOnestepSolverDrawbeadProp *drawbeadforce, const int drawbeadGroupNum);

    ONESTEPEXPORT bool OnestepUnform_SetBoundaryMesh(TOnestepSolverNode *boundarynode, TOnestepSolverBoundaryElem *boundaryelement, const int nodenum, const int elemnum);
    ONESTEPEXPORT bool OnestepUnform_SetBoundaryForce(TOnestepSolverBoundaryProp *boundaryforce, const int boundaryGroupNum);
    // end edited by zhou ping 2009.05.12

    /*06/03/2009, Xiangkui Zhang, add, start*/
    ONESTEPEXPORT bool OnestepUnform_GetBorderLoops(int *&nids, int *&loop_len, int &loop_num);
    /*06/03/2009, Xiangkui Zhang, add, end*/

    /*07/15/2012 Xiangkui Zhang, add, start*/
    ONESTEPEXPORT bool OnestepUnform_ReverseUnformSide(bool bReverse); // bReverse default value is false.
    /*07/15/2012 Xiangkui Zhang, add, end*/

    /*03/16/2016 Xiangkui Zhang, add, start*/
    ONESTEPEXPORT bool OnestepUnform_SetDiamondElementConstrain(int *de_cids, const int de_len);
    
    //Implicit condition: before this function is invoked, unform_bend_partid should be clone from unform_flatten_partid.
    ONESTEPEXPORT bool OnestepUnform_MappingMesh(unsigned long long ulModel, int target_bend_partid, int target_flattend_partid, int unform_flatten_partid,int unform_bend_partid);
    ONESTEPEXPORT bool OnestepUnform_MappingMeshEx(unsigned long long ulModel, int target_bend_partid, int target_flattend_partid, int *area_eids, int area_elen, double *area_points);
    ONESTEPEXPORT bool OnestepUnform_SetMainArea(double *mpoints, int nlen, int *meids, int elen);
    /*03/16/2016 Xiangkui Zhang, add, end*/

    /*06/30/2016 Xiangkui Zhang, add, start*/
    ONESTEPEXPORT bool OnestepUnform_EnableTopologyConsistentFlagBetweenU2T(bool flag);
    extern C_LINKAGE ONESTEPEXPORT bool OnestepUnformEnableTopologyConsistentFlagBetweenU2T(bool flag);
    /*06/30/2016 Xiangkui Zhang, add, end*/

    /*07/13/2016 Xiangkui Zhang, add, start*/
     /*Return Flag: 
     * 0-Successful; 
     * Error: Solver will be aborted.
     * 1-Part number is error: must have and only two connected mesh parts
     * 2-Output Log;
     * 3-share node ids array size is zero; snlen == 0 or share_nids == NULL;
     * 4-target region's element ids array size is zero. telen == 0 or target_eids == NULL;
     * 5-point coordinates array size is zero. vnlen == 0 or vpts == NULL;
     * 6-elements array size is zero. velen == 0 or velements == NULL;
     * 7-unknown error
    */
    ONESTEPEXPORT bool OnestepUnform_TrimLineSolve(
        double *vpts/*IO*/, int vnlen/*I*/,                                                       //points coordinates array, result will be returned by this parameter; this array's size is vneln * 3
        int *velements/*I*/, int velen/*I*/,                                                       //elements array: n1,n2,n3,n4... and n3 == n4 if element is triangle; this array's size is velen * 4
        int *target_eids/*I*/, int telen/*I*/,                                                      //target region's element ids array; this array's size is telen
        int *share_nids/*I*/, int snlen/*I*/,                                                      //share node ids array between unform region and target region (share rule: distance < tol); this array's size is sneln
        const KMAS::TOnestepSolverTrimLineParameters &para/*I*/,           // parameters: draw direction/thickness/surface type/material,
        double *ppts_of_share_nids/*I*/,                                                        // the projection points array form share node to target regions.  this array's size is sneln * 3
        int &nRetCode/*O*/                
        );
    /*Return Flag: 
     * 0-Successful; 
     * Error: Solver will be aborted.
     * 1-Part number is error: must have and only two connected mesh parts
     * 3-share node ids array size is zero; snlen == 0 or share_nids == NULL;
     * 4-target region's element ids array size is zero. telen == 0 or target_eids == NULL;
     * 5-point coordinates array size is zero. vnlen == 0 or vpts == NULL;
     * 6-elements array size is zero. velen == 0 or velements == NULL;
     * 7-unknown error
     * Warning: Solver may continue to execute
     * 20-share element ids number < unform element ids number * 40%, it seem that tol is too small, the result of solver may be bad.
     * 21-target meshes have overlap elements with current draw direction, the result of solver may be bad
     * 22-there are gaps in target region faces , the result of solver may be bad.
    */
      ONESTEPEXPORT bool OnestepUnform_CheckTrimLineSolveParameters(
          double *pts/*IO*/, int nlen/*I*/,                                                          //points coordinates array, result will be returned by this parameter; this array's size is neln * 3
          int *elements/*I*/, int elen/*I*/,                                                          //elements array: n1,n2,n3,n4... and n3 == n4 if element is triangle; this array's size is elen * 4
          int *target_eids/*I*/, int telen/*I*/,                                                      //target region's element ids array; this array's size is telen
          int *share_nids/*I*/, int snlen/*I*/,                                                      //share node ids array between unform region and target region (share rule: distance < tol); this array's size is snlen
          const KMAS::TOnestepSolverTrimLineParameters &para/*I*/,           // parameters: draw direction/thickness/surface type/material,
          double *ppts_of_share_nids/*I*/,                                                        // the projection points array form share node to target regions.  this array's size is snlen * 3
          int &nRetCode/*O*/                                                           
          );
     /*07/13/2016 Xiangkui Zhang, add, end*/

      /*08/13/2016 Xiangkui Zhang, add, start*/
      ONESTEPEXPORT bool OnestepUnform_GetShareElementIDs(
          int *&share_eids/*O*/,
          int &selen/*O*/);
      ONESTEPEXPORT bool OnestepUnform_GetShareNodeIDs(
          int *&share_nids/*O*/,
          int &snlen/*O*/);
      /*08/13/2016 Xiangkui Zhang, add, end*/
      /*09/19/2018 Xiangkui Zhang, Add, Start*/
      //Notify: This API is only applicable to stamping parts excluding overlapping areas.
      ONESTEPEXPORT bool OnestepUnform_SetBulkingCoefficient(double bulking_coefficient);
      /*09/19/2018 Xiangkui Zhang, Add, End*/

}
#undef EXPORTLIBRARY
#endif //OnestepUnformSolver_HXX_INCLUDED



