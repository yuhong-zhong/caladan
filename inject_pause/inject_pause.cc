#include <gcc-plugin.h>
#include <tree.h>
#include <gimple.h>
#include <rtl.h>
#include <emit-rtl.h>
#include <basic-block.h>
#include <tree-pass.h>
#include <context.h>
#include <function.h>
#include <stringpool.h>
#include <cgraph.h>
#include <plugin-version.h>

int plugin_is_GPL_compatible;

// Plugin name and information
static struct plugin_info pause_plugin_info = {
    .version = "1.0",
    .help = "Insert pause instruction before every instruction in cc_thread function",
};

// Pass definition
struct insert_pause_pass : rtl_opt_pass {
    insert_pause_pass(gcc::context *ctxt)
        : rtl_opt_pass(make_pass_data(), ctxt)
    {}

    // Create pass data
    static const pass_data& make_pass_data() {
        static struct pass_data data = {
            RTL_PASS,              // type
            "insert_pause",        // name
            OPTGROUP_NONE,        // optinfo_flags
            TV_NONE,              // tv_id
            0,                    // properties_required
            0,                    // properties_provided
            0,                    // properties_destroyed
            0,                    // todo_flags_start
            0                     // todo_flags_finish
        };
        return data;
    }

    // Main execution function
    virtual unsigned int execute(function *fun) override {
        basic_block bb;
        rtx_insn *insn;

        if (!fun) {
            fprintf(stderr, "Error: Function is null\n");
            return -1;
        }

        // Iterate through all basic blocks
        FOR_EACH_BB_FN(bb, fun) {
            if (!bb) continue;
            // Temporary pointer for safe iteration
            rtx_insn *insn, *curr;
            FOR_BB_INSNS_SAFE(bb, insn, curr) {
                if (insn && INSN_P(insn) && strcmp(GET_RTX_NAME(GET_CODE(insn)), "insn") == 0) {
                    // Insert pause instruction before the current instruction
                    fprintf(stderr, "inserting pause before instruction name %s\n", GET_RTX_NAME(GET_CODE(insn)));
                    emit_insn_before(gen_pause(), insn);
                }
            }
        }

        return 0;
    }

    // Gate function
    virtual bool gate(function *) override {
        tree fndecl = current_function_decl;
        if (!fndecl)
            return false;

        const char *fname = IDENTIFIER_POINTER(DECL_NAME(fndecl));
        return strcmp(fname, "cc_loop_pmyiok") == 0 || strcmp(fname, "cc_loop_seciok") == 0;
    }

    // Clone function
    virtual opt_pass* clone() override {
        return new insert_pause_pass(m_ctxt);
    }
};

// Plugin initialization
int plugin_init(struct plugin_name_args *plugin_info,
               struct plugin_gcc_version *version)
{
    // Plugin compatibility check
    if (!plugin_default_version_check(version, &gcc_version)) {
        return 1;
    }

    // Create the pass
    auto *pass = new insert_pause_pass(g);
    struct register_pass_info pass_info;

    // Register pass info
    pass_info.pass = pass;
    pass_info.reference_pass_name = "cse_local";
    // pass_info.reference_pass_name = "final";
    pass_info.ref_pass_instance_number = 1;
    pass_info.pos_op = PASS_POS_INSERT_AFTER;
    // pass_info.pos_op = PASS_POS_INSERT_BEFORE;

    // Register the pass
    register_callback(plugin_info->base_name,
                      PLUGIN_PASS_MANAGER_SETUP,
                      NULL,
                      &pass_info);

    return 0;
}
