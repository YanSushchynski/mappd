#include "cx_json_parser.hpp"
#include <cstdio>

int main(int argc, char *argv[]) {
  using namespace std::literals;
  using namespace std::json::literals;

//   constexpr auto true_val = std::json::bool_parser()("true"sv);
//   static_assert(true_val && true_val->first);

//   constexpr auto false_val = std::json::bool_parser()("false"sv);
//   static_assert(false_val && !false_val->first);

//   constexpr auto null_val = std::json::null_parser()("null"sv);
//   static_assert(null_val);

//   constexpr auto char_val = std::json::string_char_parser()("t"sv);
//   static_assert(char_val && char_val->first[0] == 't');

//   constexpr auto echar_val = std::json::string_char_parser()("\t"sv);
//   static_assert(echar_val && echar_val->first[0] == '\t');

//   {
//     constexpr auto str_val = std::json::string_parser()(R"("")"sv);
//     static_assert(str_val && str_val->first.empty());
//   }

//   {
//     constexpr auto str_val = std::json::string_parser()(R"("hello")"sv);
//     constexpr auto expected_str = "hello"sv;

//     static_assert(str_val && std::cx::equal(str_val->first.cbegin(), str_val->first.cend(), expected_str.cbegin(),
//                                             expected_str.cend()));
//   }

//   {
//     constexpr auto u = std::json::unicode_point_parser()("\\u2603"sv);
//     static_assert(u && u->first.size() == 3 && u->first[0] == static_cast<char>(0xe2) &&
//                   u->first[1] == static_cast<char>(0x98) && u->first[2] == static_cast<char>(0x83));
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("0"sv);
//     static_assert(number_val && number_val->first == 0);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("123"sv);
//     static_assert(number_val && number_val->first == 123.0);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("-123"sv);
//     static_assert(number_val && number_val->first == -123.0);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()(".123"sv);
//     static_assert(!number_val);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("0.123"sv);
//     static_assert(number_val && number_val->first == 0.123);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("456.123"sv);
//     static_assert(number_val && number_val->first == 456.123);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("456.123e1"sv);
//     static_assert(number_val && number_val->first == 456.123e1);
//   }

//   {
//     constexpr auto number_val = std::json::number_parser()("456.123e-1"sv);
//     static_assert(number_val && number_val->first == 456.123e-1);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()("true"sv);
//     static_assert(d && d->first.num_objects == 1);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()("[]"sv);
//     static_assert(d && d->first.num_objects == 1);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()("[1,2,3,4]"sv);
//     static_assert(d && d->first.num_objects == 5);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()(R"({"a":1, "b":2})"sv);
//     static_assert(d && d->first.num_objects == 5);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()(R"("a")"sv);
//     static_assert(d && d->first.string_size == 1);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()("true"sv);
//     static_assert(d && d->first.string_size == 0);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()(R"(["a", "b"])"sv);
//     static_assert(d && d->first.string_size == 2);
//   }

//   {
//     constexpr auto d = std::json::sizes_parser()(R"({"a":1, "b":2})"sv);
//     static_assert(d && d->first.string_size == 2);
//   }

//   {
//     constexpr auto sv = "true"sv;
//     constexpr auto r = std::json::extent_parser()(sv);
//     static_assert(r && std::cx::equal(r->first.cbegin(), r->first.cend(), sv.cbegin(), sv.cend()));
//   }

//   {
//     constexpr auto sv = R"("hello")"sv;
//     constexpr auto r = std::json::extent_parser()(sv);
//     static_assert(r && std::cx::equal(r->first.cbegin(), r->first.cend(), sv.cbegin(), sv.cend()));
//   }

//   {
//     constexpr auto sv = "123.456"sv;
//     constexpr auto r = std::json::extent_parser()(sv);
//     static_assert(r && std::cx::equal(r->first.cbegin(), r->first.cend(), sv.cbegin(), sv.cend()));
//   }

//   {
//     constexpr auto sv = "[1,2,3]"sv;
//     constexpr auto r = std::json::extent_parser()(sv);
//     static_assert(r && std::cx::equal(r->first.cbegin(), r->first.cend(), sv.cbegin(), sv.cend()));
//   }

//   {
//     constexpr auto sv = R"({"a":1, "b":2})"sv;
//     constexpr auto r = std::json::extent_parser()(sv);
//     static_assert(r && std::cx::equal(r->first.cbegin(), r->first.cend(), sv.cbegin(), sv.cend()));
//   }

//   {
//     constexpr auto jsv = "true"_json;
//     static_assert(jsv.to_Boolean());
//   }

//   {
//     constexpr auto jsv = "false"_json;
//     static_assert(!jsv.to_Boolean());
//   }

//   {
//     constexpr auto jsv = "null"_json;
//     static_assert(jsv.is_Null());
//   }

//   {
//     constexpr auto jsv = "123.456"_json;
//     static_assert(jsv.to_Number() == 123.456);
//   }

//   {
//     constexpr auto jsv = R"("hello")"_json;
//     static_assert(jsv.to_String() == "hello");
//   }

//   {
//     constexpr auto jsv = R"("01234567890123456789012345678901234567890123456789")"_json;
//     static_assert(jsv.to_String() == "01234567890123456789012345678901234567890123456789");
//     static_assert(jsv.string_size() == 50);
//   }

//   {
//     constexpr auto jsv = "[]"_json;
//     static_assert(jsv.array_Size() == 0);
//   }

//   {
//     constexpr auto jsv = "[1, true, 3.3423e-1]"_json;
//     static_assert(jsv[0].to_Number() == 1);
//     static_assert(jsv[1].to_Boolean());
//     // static_assert(jsv[2].to_Number() == 3.3423e-1);
//   }

//   {
//     constexpr auto jsv = "[1, [true, false], [2, 3]]"_json;
//     static_assert(jsv[0].to_Number() == 1);
//     static_assert(jsv[1][0].to_Boolean());
//     static_assert(!jsv[1][1].to_Boolean());
//     static_assert(jsv[2][0].to_Number() == 2);
//     static_assert(jsv[2][1].to_Number() == 3);
//   }

//   {
//     constexpr auto jsv = "[1, null, true, [2]]"_json;
//     static_assert(jsv.array_Size() == 4);
//     static_assert(jsv[0].to_Number() == 1);
//     static_assert(jsv[1].is_Null());
//     static_assert(jsv[2].to_Boolean());
//     static_assert(jsv[3][0].to_Number() == 2);
//   }

//   {
//     constexpr auto jsv = "[[[[[[[[[[[[1]]]]]]]]]]]]"_json;
//     static_assert(jsv[0][0][0][0][0][0][0][0][0][0][0][0].to_Number() == 1);
//   }

//   {
//     constexpr auto jsv = "[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]"_json;
//     static_assert(jsv[0].to_Number() == 1 && jsv[15].to_Number() == 16);
//   }

//   // test JSON object value parsin

//   {
//     constexpr auto jsv = R"({})"_json;
//     static_assert(jsv.object_Size() == 0);
//   }

//   {
//     constexpr auto jsv = R"({"a":1})"_json;
//     static_assert(jsv.object_Size() == 1);
//     static_assert(jsv["a"].to_Number() == 1);
//   }

//   {
//     constexpr auto jsv = R"({"a":1, "b":true, "c":2})"_json;
//     static_assert(jsv.object_Size() == 3);
//     static_assert(jsv["a"].to_Number() == 1);
//     static_assert(jsv["b"].to_Boolean());
//     static_assert(jsv["c"].to_Number() == 2);
//   }

//   {
//     constexpr auto jsv = R"({"a":{}})"_json;
//     static_assert(jsv["a"].object_Size() == 0);
//   }

//   {
//     constexpr auto jsv = R"({"a":1, "b":true, "c":["hello"]})"_json;
//     static_assert(jsv["a"].to_Number() == 1);
//     static_assert(jsv["b"].to_Boolean());
//     static_assert(jsv["c"][0].to_String() == "hello");
//   }

//   {
//     constexpr auto jsv = R"( [
//                                1 , null , true , [ 2 ] ,
//                                { "a" : 3.14 } , "hello"
//                              ] )"_json;

//     static_assert(jsv[0].to_Number() == 1);
//     static_assert(jsv[1].is_Null());
//     static_assert(jsv[2].to_Boolean());
//     static_assert(jsv[3][0].to_Number() == 2);
//     static_assert(jsv[4]["a"].to_Number() == 3.14);
//     static_assert(jsv[5].to_String() == "hello");
//   }

//   {
//     constexpr auto jsv = R"({"a":1, "b":2, "c":3, "d":4, "e":5, "f":6,
//                              "g":7, "h":8, "i":9, "j":10,"k":11,"l":12,
//                              "m":13,"n":14,"o":15,"p":16,"q":17,"r":18,
//                              "s":19,"t":20,"u":21,"v":22,"w":23,"x":24,
//                              "y":25,"z":26})"_json;

//     static_assert(jsv["z"].to_Number() == 26);
//   }

//   {
//     constexpr auto jsv = R"({"a":0, "b":1})"_json;
//     constexpr std::tuple<double, int> t{5.2, 33};

//     static_assert(std::get<int(jsv["b"].to_Number())>(t) == 33);

//     constexpr auto config = R"(
//       {
//         "feature-x-enabled": true
//       }
//     )"_json;

//     if constexpr (config["feature-x-enabled"].to_Boolean()) {

//       std::printf("feature-x-enabled\n");

//     } else {

//       std::printf("feature-x-disabled\n");
//     }
//   }

//   constexpr auto jsv1 = R"({)"_json;
//   constexpr auto jsv2 = R"([)"_json;
//   constexpr auto jsv3 = R"({"a")"_json;
//   constexpr auto jsv4 = R"({1)"_json;
//   constexpr auto jsv5 = R"({"a":1)"_json;
//   constexpr auto jsv6 = R"([1,])"_json;
}
