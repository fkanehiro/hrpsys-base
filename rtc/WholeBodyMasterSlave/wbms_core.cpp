//#include "wbms_core.h"
//
//void HumanSynchronizer::calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const hrp::dvector6& rfwin, const hrp::dvector6& lfwin, hrp::Vector3& zmp_ans){
//  hrp::Vector3 rfzmp,lfzmp;
//  const double F_H_OFFSET = 0.03;//地面から6軸センサ原点への高さ
//  if( rfwin(fz) > 1.0e-6 ){
//    rfzmp(0) = ( - rfwin(ty) - rfwin(fx) * F_H_OFFSET + rfwin(fz) * 0 ) / rfwin(fz) + rfpos(0);
//    rfzmp(1) = (   rfwin(tx) - rfwin(fy) * F_H_OFFSET + rfwin(fz) * 0 ) / rfwin(fz) + rfpos(1);
//  }
//  if( lfwin(fz) > 1.0e-6 ){
//    lfzmp(0) = ( - lfwin(ty) - lfwin(fx) * F_H_OFFSET + lfwin(fz) * 0 ) / lfwin(fz) + lfpos(0);
//    lfzmp(1) = (   lfwin(tx) - lfwin(fy) * F_H_OFFSET + lfwin(fz) * 0 ) / lfwin(fz) + lfpos(1);
//  }
//  if( rfwin(fz) > 1.0e-6 || lfwin(fz) > 1.0e-6 ){
//    zmp_ans(0) = ( rfzmp(0)*rfwin(fz) + lfzmp(0)*lfwin(fz) ) / ( rfwin(fz) + lfwin(fz));
//    zmp_ans(1) = ( rfzmp(1)*rfwin(fz) + lfzmp(1)*lfwin(fz) ) / ( rfwin(fz) + lfwin(fz));
//  }else{ zmp_ans(0) = 0; zmp_ans(1) = 0; }
//  zmp_ans(2) = 0;
//}
//void HumanSynchronizer::calcXYMarginToHull(const hrp::Vector2& check_point, const std::vector<hrp::Vector2>& hull, hrp::Vector4& margin_ans){
//  hrp::Vector4 margin_abs;
//  hrp::Vector2 cross_pt, anchor_vec[4] = {hrp::Vector2(1,0), hrp::Vector2(-1,0), hrp::Vector2(0,1), hrp::Vector2(0,-1)};//前後左右に伸ばしたアンカーとの交点を見る
//  for(int direc=0;direc<4;direc++){
//    calcCrossPointOnHull(check_point, check_point+anchor_vec[direc], hull, cross_pt);
//    margin_abs(direc) = (cross_pt - check_point).norm();
//  }
//  margin_ans(0) =  margin_abs(0);
//  margin_ans(1) = -margin_abs(1);
//  margin_ans(2) =  margin_abs(2);
//  margin_ans(3) = -margin_abs(3);
//}
//bool HumanSynchronizer::calcCrossPointOnHull(const hrp::Vector2& pt_in_start, const hrp::Vector2& pt_out_goal, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_will_cross){
//  hrp::Vector2 anchor_vec = pt_out_goal - pt_in_start;
//  for(int i=0;i<hull.size();i++){
//    int i_nxt = (i!=hull.size()-1 ? i+1 : 0);
//    hrp::Vector2 cur_pt = hull[i], nxt_pt = hull[i_nxt];
//    hrp::Vector2 cur_edge = nxt_pt - cur_pt;
//    double dBunbo = hrpVector2Cross(anchor_vec,cur_edge);
//    if( dBunbo != 0.0) {//平行の場合を外す
//      hrp::Vector2 vectorAC = hrp::Vector2(hull[i](0),hull[i](1)) - pt_in_start;
//      double dR = hrpVector2Cross(vectorAC,cur_edge) / dBunbo;
//      double dS = hrpVector2Cross(vectorAC,anchor_vec) / dBunbo;
//      if(dR > 1e-9 && dS >= 0.0 && dS <= 1.0){//dRには数値誤差が乗る
//        pt_will_cross = pt_in_start + dR * anchor_vec;
//        //dist = dR * anchor_vec.norm();
//        return true;
//      }
//    }
//  }
//  return false;
//}
//double HumanSynchronizer::calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_ans){
//  double cur_nearest_dist, ans_nearest_dist;
//  hrp::Vector2 cur_nearest_pt, ans_nearest_pt;
//  for(int i=0;i<hull.size();i++){
//    int i_nxt = (i!=hull.size()-1 ? i+1 : 0);
//    hrp::Vector2 cur_pt = hull[i], nxt_pt = hull[i_nxt];
//    hrp::Vector2 cur_edge = nxt_pt - cur_pt;
//    hrp::Vector2 tgt_pt_v = tgt_pt - cur_pt;
//    double tgt_pt_projected_length = tgt_pt_v.dot(cur_edge.normalized());
//    if(tgt_pt_projected_length > cur_edge.norm() ){//tgt_pt's nearest point is on the　i+1-th vertex
//      cur_nearest_pt = nxt_pt;
//    }else if(tgt_pt_projected_length < 0 ){//tgt_pt's nearest point is on the　i-th vertex
//      cur_nearest_pt = cur_pt;
//    }else{//tgt_pt's nearest point is on the line
//      cur_nearest_pt = cur_pt + tgt_pt_projected_length * cur_edge.normalized();
//    }
//    cur_nearest_dist = (tgt_pt - cur_nearest_pt).norm();
//    if(i==0){//set first candidate as nearest
//      ans_nearest_dist = cur_nearest_dist;
//      ans_nearest_pt = cur_nearest_pt;
//    }else if( cur_nearest_dist < ans_nearest_dist ){//update nearest candidate
//      ans_nearest_dist = cur_nearest_dist;
//      ans_nearest_pt = cur_nearest_pt;
//    }
//  }
//  pt_ans = ans_nearest_pt;
//  return ans_nearest_dist;
//}
//void HumanSynchronizer::makeConvexHullOpenCV(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
//  points.clear();
//  cvhull.clear();
//  hull_ans.clear();
//  for(int i=0;i<pts.size();i++){ points.push_back( cv::Point2f( pts[i](0), pts[i](1)) ); }
//  cv::convexHull(cv::Mat(points),cvhull,true);
//  for(int i=0;i<cvhull.size();i++){ hull_ans.push_back( hrp::Vector2( cvhull[i].x, cvhull[i].y ) );  }
//}
////void HumanSynchronizer::makeConvexHullQHull(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
////  const int dim = 2;
////  double points[dim*pts.size()];
////  for(int i=0;i<pts.size();i++){ points[dim*i] = pts[i](0); points[dim*i+1] = pts[i](1); }
////  qh_new_qhull( dim, pts.size(), points, false, "qhull ", NULL, stderr);
////  pointT *point, *pointtemp;
////  hull_ans.clear();
////  FORALLpoints hull_ans.push_back( hrp::Vector2(point[0],point[1]) );
////  qh_freeqhull(!qh_ALL);
////  //#define qh_ORIENTclock 1
////}
//bool HumanSynchronizer::isPointInHullOpenCV(const hrp::Vector2& pt, const std::vector<hrp::Vector2>& hull){
//  cvhull.clear();
//  for(int i=0;i<hull.size();i++){ cvhull.push_back( cv::Point2f( hull[i](0), hull[i](1)) ); }
//  return (cv::pointPolygonTest(cv::Mat(cvhull), cv::Point2f(pt(0),pt(1)), false) > 0);
//}
//void HumanSynchronizer::applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
//  comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
//  comacc = acc4zmp_v_filters.passFilter(comacc);
//  zmpout(0) = comin(0)-(H_cur/G)*comacc(0);
//  zmpout(1) = comin(1)-(H_cur/G)*comacc(1);
//  com_oldold = com_old;
//  com_old = comin;
//}
////void HumanSynchronizer::applyVelLimit(const HumanPose& in, HumanPose& out_old, HumanPose& out){
////  const int ns[] = {com,rf,lf,rh,lh,head}, num_ns = sizeof(ns)/sizeof(int);
////  for(int i=0;i<num_ns;i++){
////    hrp::Vector3 diff = in.P[i].p  - out_old.P[i].p;
////    for(int j=0;j<3;j++)LIMIT_MINMAX( diff(j), -MAXVEL*DT, MAXVEL*DT);
////    out.P[i].p = out_old.P[i].p + diff;
////  }
////  out_old = out;
////}
