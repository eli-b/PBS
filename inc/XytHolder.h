#ifndef XYTHOLDER_H
#define XYTHOLDER_H

#include <algorithm>
#include <list>
#include <tuple>
#include <exception>
#include <iostream>
#include <memory>  // for unique_ptr
#include <vector>

/// Stores T instances by location and time.
/// Since every timestep is sparse (most locations at each timestep are unused),
/// for each location, we map the timestep they're used in to the T instance.
template <class T>
class XytHolder {
public:
    XytHolder(int xy_size) : data(xy_size, nullptr) {}

//    // FIXME: Must do deep copy - can't have unique_ptrs pointing to same data...
//    XytHolder(const XytHolder<T>& other) : data(other.data.size(), nullptr) {
//        for (int i = 0; i < other.data.size() ; ++i) {
//            if (other.data[i] != nullptr) {
//                for (const auto& pair: *other.data[i]) {
//                    auto& [time, item] = pair;
//                    set(i, time, item.get());
//                }
//            }
//        }
//    }

    XytHolder(XytHolder<T>&& other) : count(other.count), data(std::move(other.data)) {}  // Move constructor

    // TODO: Implement an iterator and begin and end methods:
//    class Iterator {
//        XytHolder* holder;
//        int location;
//        std::list<std::tuple<int,T>>::iterator it;
//    };

    // Returns <true,requested item> or <false,nullptr> when it's missing
    std::tuple<bool, T*> get(int location_id, int t) const {
        // Linear lookup
        if (data[location_id] == nullptr)
            return std::make_tuple(false, nullptr);
        for (auto it = data[location_id]->begin(); it != data[location_id]->end() ; ++it)  {
            auto& [it_t, it_n] = *it;
            if (it_t == t) {
                return std::make_tuple(true, it_n.get());
            }
            else if (it_t > t) {
                return std::make_tuple(false, nullptr);
            }
        }
        return std::make_tuple(false, nullptr);
    }

    std::tuple<bool, int> latest_entry(int location_id) const {
        if (data[location_id] == nullptr)
            return std::make_tuple(false, -1);
        auto& [last_t, last_n] = data[location_id]->back();
        return std::make_tuple(true, last_t);
    }

    void set(int location_id, int t, T* value) {
        ++count;
        // Linear insertion
        if (data[location_id] == nullptr)
            data[location_id] = new std::list<std::tuple<int,std::unique_ptr<T>>>();
        for (auto it = data[location_id]->begin(); it != data[location_id]->end() ; ++it)  {
            auto& [it_t, it_n] = *it;
            if (it_t == t) {
                std::cout << "Unexpected re-insertion of item to XytHolder!" << std::endl;
                std::abort();
            }
            else if (it_t > t) {
                data[location_id]->insert(it, std::make_tuple(t, std::unique_ptr<T>(value)));  // inserts before the iterator
                return;
            }
        }
        data[location_id]->push_back(std::make_tuple(t, std::unique_ptr<T>(value)));
    }

    ~XytHolder() {
        clear();
    }

    void clear() {
        for (size_t i = 0; i < data.size(); ++i) {
            delete data[i];
            data[i] = nullptr;
        }
        count = 0;
    }

    int count = 0;
    std::vector<std::list<std::tuple<int,std::unique_ptr<T>>>*> data;  // nullptrs are smaller than empty std::lists
};


#endif //XYTHOLDER_H
