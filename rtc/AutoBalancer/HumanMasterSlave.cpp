#include "HumanMasterSlave.h"

void HumanSynchronizer::calcWorldZMP(const hrp::Vector3& rfpos, const hrp::Vector3& lfpos, const HumanPose::Wrench6& rfwin, const HumanPose::Wrench6& lfwin, hrp::Vector3& zmp_ans){
  hrp::Vector3 rfzmp,lfzmp;
  const double F_H_OFFSET = 0.03;//地面から6軸センサ原点への高さ
  if( rfwin.f(2) > 1.0e-6 ){
    rfzmp(0) = ( - rfwin.t(1) - rfwin.f(0) * F_H_OFFSET + rfwin.f(2) * 0 ) / rfwin.f(2) + rfpos(0);
    rfzmp(1) = (   rfwin.t(0) - rfwin.f(1) * F_H_OFFSET + rfwin.f(2) * 0 ) / rfwin.f(2) + rfpos(1);
  }
  if( lfwin.f(2) > 1.0e-6 ){
    lfzmp(0) = ( - lfwin.t(1) - lfwin.f(0) * F_H_OFFSET + lfwin.f(2) * 0 ) / lfwin.f(2) + lfpos(0);
    lfzmp(1) = (   lfwin.t(0) - lfwin.f(1) * F_H_OFFSET + lfwin.f(2) * 0 ) / lfwin.f(2) + lfpos(1);
  }
  if( rfwin.f(2) > 1.0e-6 || lfwin.f(2) > 1.0e-6 ){
    zmp_ans(0) = ( rfzmp(0)*rfwin.f(2) + lfzmp(0)*lfwin.f(2) ) / ( rfwin.f(2) + lfwin.f(2));
    zmp_ans(1) = ( rfzmp(1)*rfwin.f(2) + lfzmp(1)*lfwin.f(2) ) / ( rfwin.f(2) + lfwin.f(2));
  }else{ zmp_ans(0) = 0; zmp_ans(1) = 0; }
  zmp_ans(2) = 0;
}

void HumanSynchronizer::createSupportRegionByFootPos(const hrp::Vector3& rfin_abs, const hrp::Vector3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn, std::vector<hrp::Vector2>& hull_ans){
  std::vector<hrp::Vector2> points;
  for(int i=0;i<2;i++){
    for(int j=2;j<4;j++){
      points.push_back(hrp::Vector2(rfin_abs(0) + rf_mgn(i),    rfin_abs(1) + rf_mgn(j)));
      points.push_back(hrp::Vector2(lfin_abs(0) + lf_mgn(i),    lfin_abs(1) + lf_mgn(j)));
    }
  }
  makeConvexHullOpenCV(points, hull_ans);
}
void HumanSynchronizer::calcXYMarginToHull(const hrp::Vector2& check_point, const std::vector<hrp::Vector2>& hull, hrp::Vector4& margin_ans){
  hrp::Vector4 margin_abs;
  hrp::Vector2 cross_pt, anchor_vec[4] = {hrp::Vector2(1,0), hrp::Vector2(-1,0), hrp::Vector2(0,1), hrp::Vector2(0,-1)};//前後左右に伸ばしたアンカーとの交点を見る
  for(int direc=0;direc<4;direc++){
    calcCrossPointOnHull(check_point, check_point+anchor_vec[direc], hull, cross_pt);
    margin_abs(direc) = (cross_pt - check_point).norm();
  }
  margin_ans(0) =  margin_abs(0);
  margin_ans(1) = -margin_abs(1);
  margin_ans(2) =  margin_abs(2);
  margin_ans(3) = -margin_abs(3);
}
bool HumanSynchronizer::calcCrossPointOnHull(const hrp::Vector2& pt_in_start, const hrp::Vector2& pt_out_goal, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_will_cross){
  hrp::Vector2 anchor_vec = pt_out_goal - pt_in_start;
  for(int i=0;i<hull.size();i++){
    int i_nxt = (i!=hull.size()-1 ? i+1 : 0);
    hrp::Vector2 cur_pt = hull[i], nxt_pt = hull[i_nxt];
    hrp::Vector2 cur_edge = nxt_pt - cur_pt;
    double dBunbo = hrpVector2Cross(anchor_vec,cur_edge);
    if( dBunbo != 0.0) {//平行の場合を外す
      hrp::Vector2 vectorAC = hrp::Vector2(hull[i](0),hull[i](1)) - pt_in_start;
      double dR = hrpVector2Cross(vectorAC,cur_edge) / dBunbo;
      double dS = hrpVector2Cross(vectorAC,anchor_vec) / dBunbo;
      if(dR > 1e-9 && dS >= 0.0 && dS <= 1.0){//dRには数値誤差が乗る
        pt_will_cross = pt_in_start + dR * anchor_vec;
        //dist = dR * anchor_vec.norm();
        return true;
      }
    }
  }
  return false;
}
double HumanSynchronizer::calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const std::vector<hrp::Vector2>& hull, hrp::Vector2& pt_ans){
  double cur_nearest_dist, ans_nearest_dist;
  hrp::Vector2 cur_nearest_pt, ans_nearest_pt;
  for(int i=0;i<hull.size();i++){
    int i_nxt = (i!=hull.size()-1 ? i+1 : 0);
    hrp::Vector2 cur_pt = hull[i], nxt_pt = hull[i_nxt];
    hrp::Vector2 cur_edge = nxt_pt - cur_pt;
    hrp::Vector2 tgt_pt_v = tgt_pt - cur_pt;
    double tgt_pt_projected_length = tgt_pt_v.dot(cur_edge.normalized());
    if(tgt_pt_projected_length > cur_edge.norm() ){//tgt_pt's nearest point is on the　i+1-th vertex
      cur_nearest_pt = nxt_pt;
    }else if(tgt_pt_projected_length < 0 ){//tgt_pt's nearest point is on the　i-th vertex
      cur_nearest_pt = cur_pt;
    }else{//tgt_pt's nearest point is on the line
      cur_nearest_pt = cur_pt + tgt_pt_projected_length * cur_edge.normalized();
    }
    cur_nearest_dist = (tgt_pt - cur_nearest_pt).norm();
    if(i==0){//set first candidate as nearest
      ans_nearest_dist = cur_nearest_dist;
      ans_nearest_pt = cur_nearest_pt;
    }else if( cur_nearest_dist < ans_nearest_dist ){//update nearest candidate
      ans_nearest_dist = cur_nearest_dist;
      ans_nearest_pt = cur_nearest_pt;
    }
  }
  pt_ans = ans_nearest_pt;
  return ans_nearest_dist;
}
void HumanSynchronizer::makeConvexHullOpenCV(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
  std::vector<cv::Point2f> points,hull;
  for(int i=0;i<pts.size();i++){ points.push_back( cv::Point2f( pts[i](0), pts[i](1)) ); }
  cv::convexHull(cv::Mat(points),hull,true);
  hull_ans.clear();
  for(int i=0;i<hull.size();i++){ hull_ans.push_back( hrp::Vector2( hull[i].x, hull[i].y ) );  }
}
//void HumanSynchronizer::makeConvexHullQHull(const std::vector<hrp::Vector2>& pts, std::vector<hrp::Vector2>& hull_ans){
//  const int dim = 2;
//  double points[dim*pts.size()];
//  for(int i=0;i<pts.size();i++){ points[dim*i] = pts[i](0); points[dim*i+1] = pts[i](1); }
//  qh_new_qhull( dim, pts.size(), points, false, "qhull ", NULL, stderr);
//  pointT *point, *pointtemp;
//  hull_ans.clear();
//  FORALLpoints hull_ans.push_back( hrp::Vector2(point[0],point[1]) );
//  qh_freeqhull(!qh_ALL);
//  //#define qh_ORIENTclock 1
//}
bool HumanSynchronizer::isPointInHullOpenCV(const hrp::Vector2& pt, const std::vector<hrp::Vector2>& hull){
  std::vector<cv::Point2f> cvhull;
  for(int i=0;i<hull.size();i++){ cvhull.push_back( cv::Point2f( hull[i](0), hull[i](1)) ); }
  return (cv::pointPolygonTest(cv::Mat(cvhull), cv::Point2f(pt(0),pt(1)), false) > 0);
}
void HumanSynchronizer::applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
  comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
  const double MAXACC = 5;
  if(comacc(0)>MAXACC)comacc(0)=MAXACC;else if(comacc(0)<-MAXACC)comacc(0)=-MAXACC;
  if(comacc(1)>MAXACC)comacc(1)=MAXACC;else if(comacc(1)<-MAXACC)comacc(1)=-MAXACC;
  comacc = acc4zmp_v_filters.passFilter(comacc);
  zmpout(0) = comin(0)-(rp_wld_initpos.com(2)/G)*comacc(0);
  zmpout(1) = comin(1)-(rp_wld_initpos.com(2)/G)*comacc(1);
  if(DEBUG)fprintf(cz_log,"%f %f %f %f %f %f %f\n",(double)loop/HZ,comin(0),comin(1),rp_ref_out.zmp(0),rp_ref_out.zmp(1),zmpout(0),zmpout(1));
  com_oldold = com_old;
  com_old = comin;
}
void HumanSynchronizer::applyVelLimit(const HumanPose& in, const HumanPose& in_old, HumanPose& out){
  for(int i=0;i<3;i++){if(in.com(i) - in_old.com(i) > MAXVEL){out.com(i) = in_old.com(i) + MAXVEL;}else if(in.com(i) - in_old.com(i) < -MAXVEL){out.com(i) = in_old.com(i) - MAXVEL;}}
  for(int i=0;i<3;i++){if(in.rf(i)  - in_old.rf(i)  > MAXVEL){out.rf(i)  = in_old.rf(i)  + MAXVEL;}else if(in.rf(i)  - in_old.rf(i)  < -MAXVEL){out.rf(i)  = in_old.rf(i)  - MAXVEL;}}
  for(int i=0;i<3;i++){if(in.lf(i)  - in_old.lf(i)  > MAXVEL){out.lf(i)  = in_old.lf(i)  + MAXVEL;}else if(in.lf(i)  - in_old.lf(i)  < -MAXVEL){out.lf(i)  = in_old.lf(i)  - MAXVEL;}}
  for(int i=0;i<3;i++){if(in.rh(i)  - in_old.rh(i)  > MAXVEL){out.rh(i)  = in_old.rh(i)  + MAXVEL;}else if(in.rh(i)  - in_old.rh(i)  < -MAXVEL){out.rh(i)  = in_old.rh(i)  - MAXVEL;}}
  for(int i=0;i<3;i++){if(in.lh(i)  - in_old.lh(i)  > MAXVEL){out.lh(i)  = in_old.lh(i)  + MAXVEL;}else if(in.lh(i)  - in_old.lh(i)  < -MAXVEL){out.lh(i)  = in_old.lh(i)  - MAXVEL;}}
}
void HumanSynchronizer::applyCOMZMPXYZLock(HumanPose& tgt){
    if(!use_x){tgt.com(0) = 0;tgt.zmp(0) = 0;}
    if(!use_y){tgt.com(1) = 0;tgt.zmp(1) = 0;}
    if(!use_z){tgt.com(2) = 0;}
}
